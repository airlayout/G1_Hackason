/* Execute real input and controller functions with fake transport/time, no browser. */
'use strict';
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const path=require('node:path');
const web=path.join(__dirname,'../web');
const {DriveInput,summary,jointIndexes}=require(path.join(web,'mobile.js'));
const drive=new DriveInput();
drive.press(1,'forward');drive.press(2,'turn-left');
assert.deepEqual(drive.target(),{f:1,s:0,t:1,ratio:0.4});
drive.release(1);assert.equal(drive.target().f,0);assert.equal(drive.target().t,1);
drive.press(3,'turn-right');assert.equal(drive.target().t,0);
drive.reset();assert.equal(drive.target().t,0);
drive.press(1,'backward');drive.ratio=0.7;assert.equal(drive.target().f,-1);assert.equal(drive.target().ratio,0.7);
assert.equal(new Set([...jointIndexes('legs'),...jointIndexes('arms'),...jointIndexes('waist')]).size,29);
const sample={jetson:{state:'ok'},g1:{state:'ok'},mode:{label:'Regular Mode'},telemetry:{battery:{soc:82,age_s:1}},paused:false};
assert.equal(summary(sample,1).soc,82);
assert.equal(summary(sample,9).soc,null);
assert.equal(summary({...sample,telemetry:{battery:{soc:82,age_s:5}}},4).soc,null);
assert.equal(summary({...sample,paused:true},1).mode,'未確認');
assert.equal(summary(sample,1,true).fresh,false);
assert.equal(summary({...sample,telemetry:{battery:{soc:82,age_s:20}}},1).soc,null);
assert.equal(summary({...sample,telemetry:{battery:{soc:NaN,age_s:1}}},1).soc,null);
assert.equal(summary(null,null).connection,'確認中');
const source=fs.readFileSync(path.join(web,'ui-core.js'),'utf8');
function func(name){const match=source.match(new RegExp('(?:async )?function '+name+'\\(.*?\\n\\}\\n','s'));assert(match,'missing '+name);return match[0];}
const elements=new Map();function element(id){if(!elements.has(id))elements.set(id,{textContent:'',disabled:false,hidden:false,open:false,setAttribute(){},querySelector(){return{setAttribute(){}}},focus(){},close(){this.open=false}});return elements.get(id);}
const posts=[],beacons=[],intervals=[];
const context={console,Map,document:{hidden:false,title:''},window:{scrollTo(){},uiRouteReady:true},navigator:{sendBeacon:(url,body)=>{beacons.push([url,JSON.parse(body)]);return true}},
  $:element,moEpoch:0,moClaimBusy:false,moSession:null,moSeq:0,moTimer:null,tab:'motion',MO_MS:100,uiRouteEpoch:0,
  guard:async()=>true,moShow(){},moPost:async(kind,body)=>{posts.push([kind,body]);return{ok:true,j:{session:'one',blocked:null}}},
  moLost(){context.moSession=null;context.moTimer=null},addLog(){},setInterval:(fn,ms)=>{intervals.push([fn,ms]);return intervals.length},clearInterval(){},
  moTick(){},fetch:async()=>({}),
  TABS:['home','ops','motion','action','state','joints','audio','camera','map','tasks','dds','settings'],
  ROUTES:{home:'home',mode:'ops',remote:'motion',actions:'action',status:'state',voice:'audio',sensors:'camera',joints:'joints',map:'map',tasks:'tasks',dds:'dds',settings:'settings'},
  HASHES:{home:'home',ops:'mode',motion:'remote',action:'actions',state:'status',audio:'voice',camera:'sensors',joints:'joints',map:'map',tasks:'tasks',dds:'dds',settings:'settings'},TITLES:{},
  location:{hash:'#remote'},history:{pushState(_a,_b,hash){context.location.hash=hash}},stopCameras(){},ldView(){},dds:null,loadDds(){},renderDds(){},tpStart(){},tpStop(){},loadSettings(){},moStatus(){},tkRefresh(){},mpStart(){},mpStop(){},render(){}
};
vm.createContext(context);
for(const name of ['moSendRelease','moRelease','moClaim','setTab'])vm.runInContext(func(name),context);
(async()=>{
  // Leaving while confirmation or claim is pending never arms the next screen.
  let confirm;context.guard=()=>new Promise(resolve=>confirm=resolve);
  const pending=context.moClaim();await new Promise(setImmediate);context.setTab('home');confirm(true);await pending;
  assert.equal(posts.length,0);assert.equal(context.moSession,null);
  context.tab='motion';context.location.hash='#remote';context.guard=async()=>true;
  let reply;context.moPost=async(kind,body)=>{posts.push([kind,body]);return new Promise(resolve=>reply=resolve)};
  const delayed=context.moClaim();await new Promise(setImmediate);context.setTab('status');reply({ok:true,j:{session:'late'}});await delayed;
  assert.equal(context.moSession,null);assert.equal(intervals.length,0);assert.equal(beacons.at(-1)[1].session,'late');
  // Valid claims still work and routing away releases the specific session.
  context.tab='motion';context.location.hash='#remote';context.moPost=async()=>({ok:true,j:{session:'valid',blocked:null}});
  await context.moClaim();assert.equal(context.moSession,'valid');assert.equal(intervals.length,1);
  context.setTab('actions');assert.equal(context.moSession,null);assert.equal(beacons.at(-1)[1].session,'valid');
  for(const route of Object.keys(context.ROUTES)){context.setTab(route);assert.equal(context.tab,context.ROUTES[route]);for(const page of context.TABS)assert.equal(element('tab-'+page).hidden,page!==context.tab);}
  // Back/forward callbacks choose the route without adding a new history entry.
  let pushes=0;context.history.pushState=()=>pushes++;context.setTab('state',true);assert.equal(context.tab,'state');assert.equal(pushes,0);
  console.log('PASS: simultaneous axes, released inputs, stale/unknown data, 29 joints, full-screen routes, history, and delayed claim cancellation');
})().catch(error=>{console.error(error);process.exitCode=1});
