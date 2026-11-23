/* DOM-shaped fakes exercise both complete scripts; no browser or robot is used. */
'use strict';
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict');
const web=path.join(__dirname,'../web');
const nodes=[],ids=new Map(),selectors=new Map();
function matches(node,selector){
  if(selector.startsWith('#'))return node.id===selector.slice(1);
  if(selector.startsWith('.'))return node.className.split(' ').includes(selector.slice(1));
  const attr=selector.match(/^\[([^=\]]+)(?:="?([^"\]]+)"?)?\]$/);
  if(attr)return attr[2]===undefined?Object.hasOwn(node.attrs,attr[1]):node.attrs[attr[1]]===attr[2];
  return node.tagName===selector.toUpperCase();
}
class Element{
  constructor(tag='div',attrs={}){this.tagName=tag.toUpperCase();this.attrs=attrs;this.id=attrs.id||'';this.className=attrs.class||'';this.value=attrs.value||'';this.dataset={};for(const[k,v]of Object.entries(attrs))if(k.startsWith('data-'))this.dataset[k.slice(5).replace(/-([a-z])/g,(_,x)=>x.toUpperCase())]=v;this.children=[];this.style={};this.disabled='disabled'in attrs;this.hidden='hidden'in attrs;this.width=960;this.height=640;this.clientWidth=340;this.clientHeight=220;this.textContent='';this.listeners={};this.classList={toggle:()=>{},add:()=>{},remove:()=>{}};}
  append(...children){this.children.push(...children)}
  replaceChildren(...children){this.children=children}
  get firstElementChild(){if(!this.children.length)this.children.push(new Element());return this.children[0]}
  get tBodies(){return[this.firstElementChild]}
  get elements(){return Object.fromEntries(nodes.filter(n=>n.attrs.name).map(n=>[n.attrs.name,n]))}
  setAttribute(k,v){this.attrs[k]=v}
  removeAttribute(k){delete this.attrs[k]}
  addEventListener(n,f){(this.listeners[n]??=[]).push(f)}
  focus(){}
  close(){this.open=false}
  showModal(){this.open=true}
  querySelector(s){if(!selectors.has(s))selectors.set(s,new Element(s));return selectors.get(s)}
  querySelectorAll(s){return this.children.filter(n=>matches(n,s))}
  getContext(){return new Proxy({measureText:()=>({width:10})},{get:(o,k)=>o[k]||(()=>{})})}
  getBoundingClientRect(){return{width:340,height:220,left:0,top:0}}
}
const html=fs.readFileSync(path.join(web,'index.html'),'utf8');
for(const match of html.matchAll(/<([\w-]+)\s+([^<>]+)>/g)){
  const attrs={};for(const a of match[2].matchAll(/([\w-]+)="([^"]*)"/g))attrs[a[1]]=a[2];
  if(/\shidden(?:\s|>|$)/.test(match[2]))attrs.hidden='';
  const e=new Element(match[1],attrs);nodes.push(e);if(e.id)ids.set(e.id,e);
}
function byid(id){assert(ids.has(id),'unwired element '+id);return ids.get(id)}
const document={hidden:false,body:new Element('body'),getElementById:byid,createElement:tag=>new Element(tag),createTextNode:text=>({textContent:text}),
  querySelector(s){if(s.includes(' ')){if(!selectors.has(s))selectors.set(s,new Element(s.split(' ').at(-1)));return selectors.get(s)}return nodes.find(n=>matches(n,s))||new Element()},
  querySelectorAll(s){return s.split(',').flatMap(part=>part.includes(' ')?[]:nodes.filter(n=>matches(n,part)))},addEventListener(){}};
const requests=[],timeouts=[];
const snapshot={jetson:{state:'ok',detail:''},g1:{state:'ok',detail:''},mode:{state:'ok',label:'Regular Mode',fsm_id:501,service:'ai'},telemetry:{},control:{},sampled_at:Date.now()/1000,latency_ms:1,age_s:0,poll_interval_s:1,server:{mock:true}};
const replies={
  '/api/status':snapshot,'/api/buttons':{buttons:[],transitions:{},fallback:[1],speed:[],speed_fsm_ids:[500,501]},
  '/api/features':{features:[],planned_modes:[],planned_actions:[],actions:[]},'/api/audio/volume':{volume:40},
  '/api/scenarios':{scenarios:{ok:'正常'},current:'ok'},'/api/tasks':{tasks:[]},'/api/settings':{settings:{},labels:{}},
  '/api/dds':{topics:[],services:[]},'/api/topics':{enabled:false,supported_types:[],publish:[],subscribed:[]},'/api/topics/received':{items:[]},
  '/api/motion/status':{state:'idle',operator:false,blocked:null,limits:{vx:[-.2,.3],vy:[-.2,.2],wz:[-.4,.4]}},
  '/api/cameras':{enabled:false,cameras:{}},'/api/camera/status':{running:false,holders:[],foreign:[]},'/api/lidar/status':{running:false},
  '/api/locations':{locations:[]},'/api/nav/state':{available:false,heartbeat:{enabled:false}}
};
const context={console,document,navigator:{sendBeacon:()=>true},localStorage:{getItem:()=>null},performance:{now:()=>100},AbortController,Float32Array,Image:class{},URL,Blob,
  location:{hash:'#home',host:'localhost'},history:{pushState(_a,_b,hash){context.location.hash=hash},replaceState(_a,_b,hash){context.location.hash=hash}},
  setInterval:()=>1,clearInterval(){},setTimeout:f=>{timeouts.push(f);return timeouts.length},clearTimeout(){},addEventListener(){},scrollTo(){},
  fetch:async(url,options={})=>{requests.push([url,options.method||'GET']);const payload=replies[url]||{};return{ok:true,status:200,json:async()=>payload}},
};
context.window=context;vm.createContext(context);
(async()=>{
  for(const name of ['ui-core.js','mobile.js'])vm.runInContext(fs.readFileSync(path.join(web,name),'utf8'),context,{filename:name});
  await new Promise(setImmediate);
  assert.equal(byid('ui-title').textContent,'G1 Control');
  for(const route of ['mode','remote','actions','status','joints','voice','sensors','dds','settings','tasks','map','home']){
    vm.runInContext(`setTab('${route}')`,context);await new Promise(setImmediate);
  }
  assert.equal(byid('ui-connection').textContent,'接続中');
  assert(requests.every(([url,method])=>method==='GET'||url==='/api/nav/presence'),'screen entry must not claim, start devices, or send commands');
  assert(!requests.some(([url])=>url.startsWith('/api/motion/claim')));
  console.log('PASS: both complete scripts boot, all 12 screens initialize, and navigation sends no robot/device/task commands');
})().catch(error=>{console.error(error);process.exitCode=1});
