const $=id=>document.getElementById(id);
const FRESH_S=2.5, STALE_S=8, GIVEUP_S=15, POLL_MS=1000;
const ST_LABEL={ok:'接続中',down:'未接続',unknown:'未確認',stale:'更新停止',checking:'確認中',stopped:'停止',paused:'確認停止'};
let S=null, recvAt=0, fails=0, busy=false, pending=null, buttons=[], transitions={}, fallbackTo=[1], speedFsm=[], logs=[], featuresLoaded=false;
const RECONNECT_ON=`サーバーが止まっても、この画面は ${POLL_MS/1000} 秒ごとに再試行し、復帰すると自動で更新されます。`;
const RECONNECT_OFF='停止中：サーバーは G1 を確認しません。最後に取得した値をそのまま表示します。';
let lastKey='',lastDiag='';

const ageNow=()=>S&&S.age_s!=null?S.age_s+(performance.now()-recvAt)/1000:null;
const clock=t=>new Date(t*1000).toLocaleTimeString('ja-JP',{hour12:false});
const ago=s=>s<60?`${s.toFixed(0)} 秒前`:`${Math.floor(s/60)} 分 ${Math.floor(s%60)} 秒前`;
const U=(st,detail)=>({st,detail});

/* 3 段の接続状態。上流が切れていれば下流は「未確認」にする（見えていないものを接続中と言わない） */
function nodes(){
  const server=U(fails>=2?'stopped':(S?'ok':'checking'),'');
  if(server.st==='stopped'){const d='サーバー停止のため未確認';return{server,jetson:U('unknown',d),g1:U('unknown',d)}}
  if(!S||S.sampled_at==null)return{server,jetson:U('checking','初回の確認中'),g1:U('checking','初回の確認中')};
  if(S.paused){const d='自動確認を停止中（最後の確認は '+(S.sampled_at?clock(S.sampled_at):'なし')+'）';return{server,jetson:U('paused',d),g1:U('paused',d)}}
  const age=ageNow();
  if(age>STALE_S){const d=`最後に確認できてから ${age.toFixed(0)} 秒`;return{server,jetson:U('stale',d),g1:U('stale',d)}}
  return{server,jetson:U(S.jetson.state,S.jetson.detail),g1:U(S.g1.state,S.g1.detail)};
}

function diagnose(n){
  const code=t=>{const c=document.createElement('code');c.textContent=t;return c};
  const out=[];
  if(n.server.st==='stopped')out.push('コンソールサーバーが動いていません（または応答がありません）。操作用 PC のターミナルで ',code('python3 Console/server.py'),' を起動してください。この画面は自動で再接続します。');
  else if(n.jetson.st==='paused')out.push('自動確認・再接続を停止しています。再開するか「今すぐ 1 回確認」を押すと確認します。');
  else if(n.server.st==='checking')out.push('サーバーに問い合わせています…');
  else if(n.jetson.st==='checking')out.push('サーバーは起動しました。G1 側の初回確認中です（数秒かかります）。');
  else if(n.jetson.st==='stale')out.push('サーバーは応答していますが、G1 側の確認が止まっています。通信ヘルパーが詰まっている可能性があります（',n.jetson.detail,'）。');
  else if(n.jetson.st==='down')out.push('G1 との通信ヘルパー（Jetson 内）が応答しません：'+n.jetson.detail+'。ネットワークケーブル（eth0）と DDS の設定を確認してください。');
  else if(n.g1.st==='down')out.push('Jetson には繋がっていますが、G1 のモーションサービスが応答しません。G1 の電源と起動完了を確認してください。（'+n.g1.detail+'）');
  else out.push('正常：サーバー → Jetson → G1 まで届いています。');
  const key=out.map(x=>typeof x==='string'?x:x.textContent).join('');
  if(key===lastDiag)return;  /* 同じ文面なら DOM を触らない（ちらつき防止） */
  lastDiag=key;
  $('diag').className=out[0].startsWith('正常')?'ok':'';
  $('diag').replaceChildren(...out.map(x=>typeof x==='string'?document.createTextNode(x):x));
}

function renderChain(n){
  /* コンソールは Jetson 上で動かす前提（Mac 側 server.py は廃止）。ヘルパー（Jetson 内）が不調なら G1 も見えないので、G1 ノードにその状態を出す */
  const view={server:n.server,g1:n.jetson.st==='ok'?n.g1:n.jetson};
  for(const k of ['server','g1']){
    const el=$(k+'-st'),cls='node-st s-'+view[k].st,txt=ST_LABEL[view[k].st];
    if(el.className!==cls)el.className=cls;
    if(el.textContent!==txt)el.textContent=txt;
    $('n-'+k).classList.toggle('is-bad',view[k].st==='down'||view[k].st==='stopped');
    if($('n-'+k).title!==(view[k].detail||''))$('n-'+k).title=view[k].detail||'';
  }
  $('server-sub').textContent=location.host;
  const paused=!!(S&&S.paused);
  $('mon-toggle').textContent=paused?'再開する':'停止する';
  $('mon-once').hidden=!paused;
  $('reconnect-note').textContent=paused?RECONNECT_OFF:RECONNECT_ON;
}

/* ヘッダーのカード。値は ● 正常 / ▲ 注意 / ○ 不明 の記号つき。判定はサーバー（S.control / S.mode）。
   腕は状態側 lowstate の mode で、指令側 lowcmd の mode ではない */
function setStat(id,cls,text,detail){
  const el=$(id);el.className='ctl '+cls;el.textContent=text;
  const d=$(id+'-d');if(d)d.textContent=detail||'';
}
/* リモコンの受信。rt/wirelesscontroller の最終受信からの秒数。途絶・再開は操作履歴に残す（受け付けなくなる原因調べ用） */
const RC_LIVE_S=3;let rcWas=null;
function renderRemote(T){
  const r=T&&T.remote,age=r&&r.age_s!=null?r.age_s+(performance.now()-recvAt)/1000:null;
  if(age==null){setStat('st-rc','c-mute',T?'受信なし':'—',T?'電源・G1 との接続を確認':'');return}
  const live=age<=RC_LIVE_S;
  setStat('st-rc',live?'c-ok':'c-warn',live?'受信中':'途絶 '+Math.round(age)+' 秒',live?(r.keys?'キー入力あり':'入力なし'):'電源・G1 との接続を確認');
  if(rcWas!==null&&rcWas!==live)addLog(live?'リモコン: 受信を再開':'リモコン: 受信が途絶（最後の受信から '+Math.round(age)+' 秒）',live?'ok':'bad');
  rcWas=live;
}
function renderStats(m,c){
  const none=['c-mute','—',''];
  const svc=!m?none:m.state==='ok'?['c-ok','稼働中','バランス・歩行を制御中']
    :m.state==='debug'?['c-warn','解放中（停止）','バランス・歩行・リモコンは効かない']:['c-mute','不明',m.detail||''];
  const dbg=!m?none:m.state==='ok'?['c-ok','いいえ','通常モード']
    :m.state==='debug'?['c-warn','はい','低レベル指令を直接送れる状態。このコンソールからモード切替不可']:['c-mute','不明',''];
  setStat('st-svc',...svc);setStat('st-dbg',...dbg);
  const fsm=$('st-fsm');fsm.textContent=m&&m.state==='ok'&&m.fsm_id!=null?String(m.fsm_id):'—';
  $('st-fsm-d').textContent=m&&m.state==='ok'?m.label:'';
  const L=c&&c.legs,A=c&&c.arm;
  const legs=!L?none:L.state==='autonomous'?['c-ok','自律',m&&m.label||'']
    :L.state==='not_autonomous'?['c-warn','自律ではない',m&&m.state==='debug'?'デバッグモード':(m&&m.label||'')]:['c-mute','不明','モードを取得できていません'];
  setStat('st-legs',...legs);
  const nm=A?`${A.enabled_count}/${A.total}`:'';
  const arm=!A?none:A.state==='enabled'?['c-ok','有効 '+nm,A.commanded?'指令を受信中':'指令なし']
    :A.state==='disabled'?['c-warn','無効 '+nm,'Zero Torque']:A.state==='partial'?['c-warn','一部のみ有効 '+nm,'']
    :['c-mute','不明','関節の状態を受信できていません'];
  setStat('st-arm',...arm);
}

function renderMode(n){
  const live=n.jetson.st==='ok'&&n.g1.st==='ok'&&S.mode;
  const m=live?S.mode:null;
  const mode=$('mode');
  if(m&&m.state==='ok'){mode.className='m-ok';mode.textContent=m.label;$('mode-sub').textContent=`ID ${m.fsm_id} · service=${m.service}`}
  else if(m&&m.state==='debug'){mode.className='m-debug';mode.textContent='デバッグモード';$('mode-sub').textContent=''}
  else if(m&&m.state==='unknown'){mode.className='m-none';mode.textContent='状態不明';$('mode-sub').textContent=m.detail||''}
  else{
    mode.className='m-none';
    mode.textContent=n.server.st==='checking'||n.jetson.st==='checking'?'確認中…':(n.jetson.st==='paused'?'確認停止中':(n.jetson.st==='down'||n.g1.st==='down'?'未接続':'不明'));
    const lm=S&&S.last_mode;
    $('mode-sub').textContent=lm?`最後に確認できたモード：${lm.label}${lm.fsm_id!=null?`（ID ${lm.fsm_id}）`:''} · ${clock(lm.at)}（${ago(Math.max(0,Date.now()/1000-lm.at))}）`:'';
  }
  renderStats(m,live?S.control:null);
  renderRemote(live?S.telemetry:null);
  const age=ageNow(),f=$('st-last'),fd=$('st-last-d');
  $('st-lat').textContent=live&&S.latency_ms!=null?`${S.latency_ms} ms`:'—';
  if(n.jetson.st==='paused'){f.className='sv';f.textContent=S.sampled_at?clock(S.sampled_at):'—';fd.textContent=S.sampled_at?'自動確認の停止中のため更新しません':'まだ取得していません'}
  else if(age==null){f.className='sv';f.textContent='—';fd.textContent='まだ取得できていません'}
  else{
    f.className='sv '+(age<=FRESH_S?'f-fresh':(age<=STALE_S?'f-slow':'f-stale'));
    f.textContent=clock(S.sampled_at);
    fd.textContent=`${age.toFixed(1)} 秒前`+(n.server.st==='stopped'?' · サーバー停止中（最後に受信した値）':'');
  }
}

function switchBlockReason(n){
  if(n.server.st==='stopped')return 'コンソールサーバー停止中';
  if(n.server.st==='checking'||n.jetson.st==='checking')return '確認中';
  if(n.jetson.st==='paused')return '自動確認を停止中';
  if(n.jetson.st==='stale')return '状態が更新されていません';
  if(n.jetson.st==='down')return 'Jetson 未接続';
  if(n.g1.st==='down')return 'G1 未接続';
  if(S.mode.state==='debug')return 'デバッグモード中は切り替え不可';
  if(S.mode.state!=='ok')return '現在のモードを取得できません';
  return '';
}

function renderButtons(n){
  const why=switchBlockReason(n);
  $('why').textContent=why?'切り替え無効：'+why:'';
  const cur=!why?S.mode.fsm_id:null;
  /* 押せるのは、現在のモードから遷移できるものだけ（公式の遷移図）。現在値が表に無ければ Damping だけ */
  const ok=cur==null?[]:(transitions[cur]||fallbackTo);
  document.querySelectorAll('#btns button').forEach(b=>{
    const id=Number(b.dataset.id),isCur=id===cur,reach=ok.includes(id);
    b.classList.toggle('current',isCur);
    b.disabled=!!why||busy||!reach;
    b.title=why||isCur?'':reach?'':'現在のモードからは直接切り替えられません（Damping を経由）';
  });
  const walking=cur!=null&&speedFsm.includes(cur);
  document.querySelectorAll('#speed-btns button').forEach(b=>{
    b.disabled=!walking||busy;
    b.title=walking?'':'Regular Mode（歩行）のときだけ送れます';
  });
}

/* 「送信した」と「目標のモードに着いた」は別。着くまでを見せる（この間もボタンは押せる＝緊急のダンピングを塞がない） */
function renderPending(){
  const el=$('pending');
  if(!pending){el.hidden=true;return}
  const elapsed=(performance.now()-pending.at)/1000;
  const cur=S&&S.mode&&S.mode.state==='ok'?S.mode.fsm_id:null;
  if(cur===pending.id){addLog(`「${pending.label}」に到達（${elapsed.toFixed(1)} 秒）`,'ok');pending=null;el.hidden=true;return}
  if(elapsed>GIVEUP_S){addLog(`「${pending.label}」に ${GIVEUP_S} 秒経っても到達せず（現在：${S&&S.mode?S.mode.label:'不明'}）`,'bad');pending=null;el.hidden=true;return}
  el.hidden=false;
  el.textContent=`「${pending.label}」へ遷移中… 現在：${S&&S.mode?S.mode.label:'不明'} · ${elapsed.toFixed(0)} 秒経過`;
}

function addLog(text,cls){
  logs=[{t:new Date().toLocaleTimeString('ja-JP',{hour12:false}),text,cls},...logs].slice(0,30);
  $('log').replaceChildren(...logs.map(l=>{
    const li=document.createElement('li'),t=document.createElement('span'),s=document.createElement('span');
    t.className='t';t.textContent=l.t;s.className=l.cls||'';s.textContent=l.text;li.append(t,s);return li;
  }));
}

const JOINT_NAMES=['左股 pitch','左股 roll','左股 yaw','左膝','左足首 pitch','左足首 roll',
 '右股 pitch','右股 roll','右股 yaw','右膝','右足首 pitch','右足首 roll',
 '腰 yaw','腰 roll','腰 pitch',
 '左肩 pitch','左肩 roll','左肩 yaw','左肘','左手首 roll','左手首 pitch','左手首 yaw',
 '右肩 pitch','右肩 roll','右肩 yaw','右肘','右手首 roll','右手首 pitch','右手首 yaw'];
const RAD=180/Math.PI;
const f=(v,d=2)=>v==null||!isFinite(v)?'—':Number(v).toFixed(d);

function setKV(id,rows,live){
  const el=$(id);el.className='kv'+(live?'':' stale');el.textContent='';
  rows.forEach(([k,v])=>{const dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=k;dd.textContent=live?v:'—';el.append(dt,dd)});
}

const JSON_S=v=>typeof v==='string'?v:JSON.stringify(v);
function renderSystem(T,live){
  const sy=live&&T.system,mb=live&&T.mainboard;
  const temps=sy&&sy.temps?Object.entries(sy.temps):[];
  const hot=temps.reduce((a,b)=>b[1]>a[1]?b:a,['',-1]);
  setKV('sys-kv',[
    ['Jetson 負荷（1/5/15 分）',sy&&sy.load?`${sy.load.join(' / ')}（${sy.cpus} コア）`:'—'],
    ['メモリ空き',sy&&sy.mem_total_mb?`${sy.mem_avail_mb} / ${sy.mem_total_mb} MB`:'—'],
    ['Jetson 最高温',temps.length?`${f(hot[1],1)}℃（${hot[0]}）`:'—'],
    ['ディスク空き',sy&&sy.disk_free_gb!=null?`${sy.disk_free_gb} GB`:'—'],
    ['Jetson 稼働時間',sy&&sy.uptime_s!=null?`${(sy.uptime_s/60).toFixed(0)} 分`:'—'],
    ['メインボード温度',mb?mb.temperature.filter(t=>t>0).map(t=>`${t}℃`).join(' / ')||'—':'—'],
    ['メインボード value（生）',mb?mb.value.slice(0,3).map(v=>f(v,1)).join(' / '):'—'],
    ['メインボード state（生）',mb?JSON.stringify(mb.state.slice(0,3)):'—']],!!(sy||mb));
}
function renderMisc(T,live){
  const rows=[];
  const r=live&&T.remote,e=live&&T.estop;
  rows.push(['無線リモコン入力',r?`lx ${f(r.lx)} ly ${f(r.ly)} rx ${f(r.rx)} ry ${f(r.ry)} keys ${r.keys}`:'受信なし']);
  rows.push(['非常停止イベント',e?JSON_S(Object.fromEntries(Object.entries(e).filter(([k])=>k!=='age_s'))):'受信なし']);
  const st=live&&T.strings?T.strings:{};
  ['rt/servicestate','rt/servicestateactivate','rt/multiplestate','rt/selftest','rt/lf/battery_alarm','rt/rtc/state','rt/arm/action/state','rt/public_network_status','rt/gpt_state'].forEach(k=>{
    rows.push([k.replace('rt/',''),st[k]?JSON_S(st[k].value):'受信なし']);
  });
  setKV('misc-kv',rows,live);
  const tb=document.querySelector('#rates tbody');tb.textContent='';
  Object.entries(live&&T.ages?T.ages:{}).sort((a,b)=>(a[1]==null)-(b[1]==null)||(a[1]||0)-(b[1]||0)).forEach(([k,v])=>{
    const tr=document.createElement('tr');[k,v==null?'—':f(v,1)].forEach(x=>{const td=document.createElement('td');td.textContent=x;tr.append(td)});tb.append(tr);
  });
}

function renderTelemetry(n){
  const T=S&&S.telemetry,age=ageNow();
  const live=!!T&&n.jetson.st==='ok'&&n.g1.st==='ok'&&age!=null&&age<=STALE_S;
  const b=live&&T.battery,i=live&&T.imu,o=live&&T.odom,j=live&&T.joints;
  const soc=$('bat-soc'),bar=$('bat-bar');
  if(b){
    soc.textContent=`${b.soc}%`;soc.style.color='';
    bar.className=b.soc<=10?'crit':(b.soc<=20?'low':'');bar.firstElementChild.style.width=`${Math.max(0,Math.min(100,b.soc))}%`;
  }else{soc.textContent='—';soc.style.color='';bar.className='';bar.firstElementChild.style.width='0'}
  const volts=b?b.voltage.filter(v=>v>0):[],temps=b?b.temperature.filter(t=>t>0):[];
  setKV('bat-kv',[
    ['電圧（推定）',volts.map(v=>`${f(v/1000,2)} V`).join(' / ')||'—'],
    ['電流（推定）',b?`${f(b.current/1000,2)} A（${b.current<0?'放電':'充電'}）`:'—'],
    ['温度',temps.length?temps.map(t=>`${t}℃`).join(' / '):'—'],
    ['更新',b?`${f(b.age_s,1)} 秒前に受信`:'—']],!!b);
  const d=i?i.rpy.map(v=>v*RAD):[];
  setKV('imu-kv',[
    ['ロール／ピッチ／ヨー',i?`${f(d[0],1)}° / ${f(d[1],1)}° / ${f(d[2],1)}°`:'—'],
    ['角速度 [rad/s]',i?i.gyro.map(v=>f(v)).join(' / '):'—'],
    ['加速度 [m/s²]',i?i.accel.map(v=>f(v)).join(' / '):'—'],
    ['IMU 温度',i?`${i.temperature}℃`:'—'],
    ['2 つ目の IMU（ロール／ピッチ／ヨー）',i&&T.imu2?T.imu2.rpy.map(v=>f(v*RAD,1)+'°').join(' / '):'—']],!!i);
  setKV('odom-kv',[
    ['位置 x / y / z [m]',o?o.position.map(v=>f(v)).join(' / '):'—'],
    ['速度 x / y / z [m/s]',o?o.velocity.map(v=>f(v)).join(' / '):'—'],
    ['ヨー角速度 [rad/s]',o?f(o.yaw_speed):'—'],
    ['胴体の高さ [m]',o?f(o.body_height):'—'],
    ['mode / gait / error',o?`${o.mode} / ${o.gait_type} / ${o.error_code}`:'—']],!!o);
  renderSystem(T,live);renderMisc(T,live);
  const tb=document.querySelector('#joints tbody');tb.textContent='';
  if(j){
    let hot=0,hotI=0;
    j.items.forEach((m,k)=>{
      const t=Math.max(...m.temperature);if(t>hot){hot=t;hotI=k}
      const tr=document.createElement('tr');
      const c=T.lowcmd&&T.lowcmd.items[k];
      [JOINT_NAMES[k]||`#${k}`,f(m.q*RAD,1),f(m.dq),f(m.tau),String(t),c?f(c.q*RAD,1):'—',c?f(c.kp,0):'—',c?f(c.kd,1):'—',m.mode==null?'—':String(m.mode),c&&c.mode!=null?String(c.mode):'—'].forEach(x=>{const td=document.createElement('td');td.textContent=x;tr.append(td)});
      tb.append(tr);
    });
    const lost=j.items.filter(m=>m.lost!==0).length;
    $('joint-sum').textContent=`最高温 ${hot}℃（${JOINT_NAMES[hotI]||'#'+hotI}）· motorstate が 0 以外の関節 ${lost} 個 · mode_machine ${j.mode_machine} · 指令 ${T.lowcmd?'あり':'受信なし'}`;
  }else $('joint-sum').textContent=T&&live?'関節の値を受信できていません':'—（未接続または古いデータ）';
}

// ---- 音声・LED ----
let originalVolume=null,audioReady=false,volumeTried=false;
function setRes(id,text,cls){const e=$(id);e.textContent=text;e.className='res '+(cls||'')}
async function audioPost(kind,body){
  const r=await fetch('/api/audio/'+kind,{method:'POST',body:JSON.stringify(body)});
  let j={};try{j=await r.json()}catch(_){}
  return{ok:r.ok,j,status:r.status};
}
function stopCameras(){document.querySelectorAll('#cams img').forEach(i=>i.removeAttribute('src'))}  // タブを離れたら配信接続を閉じる
async function loadCameras(){
  try{
    const epoch=uiRouteEpoch;
    const r=await (await fetch('/api/cameras')).json();
    if(epoch!==uiRouteEpoch||tab!=='camera'||window.uiSensor&&window.uiSensor!=='camera')return;
    if(!r.enabled){$('cam-note').textContent='カメラ未設定です。server.py を --camera-url 付きで起動してください。';return}
    $('cams').replaceChildren(...Object.entries(r.cameras).map(([id,label])=>{
      const box=document.createElement('div'),cap=document.createElement('b'),img=document.createElement('img');
      cap.textContent=label;img.src='/camera/'+id;img.alt=label+'（映像なし）';img.style.cssText='width:100%;display:block;background:#000';
      img.onerror=()=>{box.hidden=true;  // 未接続のカメラは枠ごと隠す
        if([...$('cams').children].every(b=>b.hidden))$('cam-note').textContent='映像がありません。上の「起動」を押したか、カメラが接続されているかを確認してください。'};
      box.append(cap,img);return box}));
    $('cam-note').textContent='Jetson の camera_stream.py の映像を中継しています。映らないときは Jetson 側で起動しているか、カメラが USB に刺さっているかを確認。';
  }catch(e){$('cam-note').textContent='カメラ情報を取得できません: '+e}
}
function renderCamProc(j){
  const own=j.holders.filter(h=>h.own).length,foreign=j.foreign.length;
  $('cp-state').textContent=(j.running?'camera_stream 起動中 (pid='+j.pid+')':'camera_stream 停止中')+
    (foreign?' ／ 他のプロセスが '+foreign+' 件掴んでいます':' ／ 他に掴んでいるプロセスなし');
  for(const p of ['cp','cv']){$(p+'-start').disabled=j.running;$(p+'-stop').disabled=!j.running}
  $('cv-state').textContent=j.running?'配信中':'停止中';
  const rows=j.holders.map(h=>{const tr=document.createElement('tr');
    [h.device,h.pid,h.user,h.cmd+(h.own?'（自前）':'')].forEach(v=>{const td=document.createElement('td');td.textContent=v;tr.append(td)});return tr});
  if(!rows.length){const tr=document.createElement('tr'),td=document.createElement('td');td.colSpan=4;td.textContent='映像デバイスを掴んでいるプロセスはありません';tr.append(td);rows.push(tr)}
  $('cp-table').tBodies[0].replaceChildren(...rows);
}
async function camProc(action){
  const get=action==='status';
  if(!get&&!await guard('camera',{quiet:true,title:'カメラ配信を'+(action==='start'?'起動':'停止')+'する',lines:['Jetson 上のカメラ配信プロセスを'+(action==='start'?'起動':'停止')+'します','起動・停止の結果は、画面の状態表示と映像で確かめてください']}))return;setRes('cp-res',get?'確認中…':(action==='start'?'起動中…':'停止中…'));
  ['cp-refresh','cp-start','cp-stop','cv-start','cv-stop'].forEach(i=>$(i).disabled=true);
  try{
    const r=await fetch('/api/camera/'+(get?'status':action),{method:get?'GET':'POST'});
    const j=await r.json();
    if(!r.ok){setRes('cp-res',j.error||('失敗 '+r.status),'bad');$('cp-refresh').disabled=false;return}
    renderCamProc(j);setRes('cp-res',j.message||'');
    if(!get&&tab==='camera'&&(!window.uiSensor||window.uiSensor==='camera'))loadCameras();  // 起動・停止後は映像の接続を張り直す
  }catch(e){setRes('cp-res','取得できません: '+e,'bad')}
  $('cp-refresh').disabled=false;
}
$('cv-start').onclick=()=>camProc('start');$('cv-stop').onclick=()=>camProc('stop');
$('cp-refresh').onclick=()=>camProc('status');$('cp-start').onclick=()=>camProc('start');$('cp-stop').onclick=()=>camProc('stop');
/* ---- lidar（点群）: 起動・停止と 2D 投影の表示。タブを離れたら取得を止める ---- */
const LD_POLL_MS=120, LD_RETRY_MS=1500, LD_VIEW0={yaw:0.6,pitch:1.1,scale:70}, LD_RINGS_M=[1,2,3,5,8,12];
let ld={...LD_VIEW0,frames:[],seq:null,timer:null,running:false,times:[]};
const LD_COLORS=16;  // 高さの色を 16 段に分け、段ごとに 1 回で塗る（重ねた数万点でも軽く）
function ldDraw(){
  const c=$('ld-canvas'),g=c.getContext('2d'),w=c.width,h=c.height,cx=w/2,cy=h/2;
  g.clearRect(0,0,w,h);
  const cy_=Math.cos(ld.yaw),sy_=Math.sin(ld.yaw),cp=Math.cos(ld.pitch),sp=Math.sin(ld.pitch);
  const proj=(x,y,z)=>{const x1=x*cy_-y*sy_,y1=x*sy_+y*cy_;return[cx+x1*ld.scale,cy-(y1*sp+z*cp)*ld.scale]};
  g.strokeStyle='rgba(180,190,200,.25)';g.fillStyle='rgba(200,210,220,.7)';g.lineWidth=1;g.font='22px sans-serif';
  for(const r of LD_RINGS_M){g.beginPath();for(let i=0;i<=64;i++){const a=i/64*2*Math.PI,[px,py]=proj(r*Math.cos(a),r*Math.sin(a),0);i?g.lineTo(px,py):g.moveTo(px,py)}g.stroke();
    const[lx,ly]=proj(r,0,0);g.fillText(r+' m',lx+4,ly-4)}
  for(const [vx,vy,col,name] of [[1,0,'#ef5350','x'],[0,1,'#66bb6a','y']]){const[a0,b0]=proj(0,0,0),[a1,b1]=proj(vx,vy,0);g.strokeStyle=col;g.fillStyle=col;g.lineWidth=2;g.beginPath();g.moveTo(a0,b0);g.lineTo(a1,b1);g.stroke();g.fillText(name,a1+4,b1)}
  const zs=[];for(const p of ld.frames)for(let i=2;i<p.length;i+=4)zs.push(p[i]);
  if(!zs.length)return;
  zs.sort((a,b)=>a-b);
  const lo=zs[Math.floor(zs.length*.05)],hi=zs[Math.floor(zs.length*.95)],span=Math.max(hi-lo,.1);
  for(let k=0;k<LD_COLORS;k++){
    g.fillStyle='hsl('+Math.round(240-k/(LD_COLORS-1)*240)+',85%,60%)';g.beginPath();
    for(const p of ld.frames)for(let i=0;i<p.length;i+=4){
      if(Math.min(LD_COLORS-1,Math.max(0,Math.floor((p[i+2]-lo)/span*LD_COLORS)))!==k)continue;
      const[px,py]=proj(p[i],p[i+1],p[i+2]);g.rect(px-1,py-1,2.5,2.5)}
    g.fill();
  }
}
function ldStats(points,ageMs){
  const now=performance.now();ld.times=ld.times.filter(t=>now-t<3000);ld.times.push(now);
  const hz=ld.times.length>1?(ld.times.length-1)/((now-ld.times[0])/1000):0;
  const total=ld.frames.reduce((n,f)=>n+f.length/4,0);
  $('ld-stats').textContent='1 枚 '+points+' 点 × '+ld.frames.length+' 枚重ね＝'+total+' 点 ／ '+hz.toFixed(1)+' fps ／ 遅れ '+ageMs+' ms';
}
function ldQuery(){const[v,m]=$('ld-density').value.split(',');return '?voxel='+v+'&max='+m}
async function ldTick(){
  if(!ld.running)return;
  let wait=LD_POLL_MS;
  try{
    const r=await fetch('/lidar'+ldQuery(),{cache:'no-store'});
    if(!r.ok){let m='';try{m=(await r.json()).error}catch(_){}throw new Error(m||('HTTP '+r.status))}
    const buf=await r.arrayBuffer(),seq=r.headers.get('X-Lidar-Seq');
    if(!ld.running||tab!=='camera'||window.uiSensor!=='lidar')return;
    if(buf.byteLength%16)throw new Error('点群の形式が不正です');
    if(seq!==ld.seq){  // 同じフレームを二重に重ねない
      ld.seq=seq;const keep=Number($('ld-stack').value);
      ld.frames=[...ld.frames,new Float32Array(buf)].slice(-keep);
      ldStats(buf.byteLength/16,r.headers.get('X-Lidar-Age-Ms')||'?');ldDraw();
    }
    setRes('ld-res','');
  }catch(e){$('ld-stats').textContent='点群を取得できません';setRes('ld-res',String(e.message||e),'bad');wait=LD_RETRY_MS}
  if(ld.running)ld.timer=setTimeout(ldTick,wait);
}
function ldView(on){
  ld.running=on;clearTimeout(ld.timer);
  if(on)ldTick();else{ld.frames=[];ld.seq=null;ld.times=[];$('ld-stats').textContent='停止中';ldDraw()}
}
$('ld-stack').onchange=()=>{ld.frames=ld.frames.slice(-Number($('ld-stack').value));ldDraw()};
$('ld-density').onchange=()=>{ld.frames=[];ld.seq=null;ldDraw()};  // 密度の違う点群を混ぜない
function renderLidarProc(j){
  $('ld-state').textContent=j.running?'lidar_stream 起動中 (pid='+j.pid+')':'lidar_stream 停止中';
  $('ld-start').disabled=j.running;$('ld-stop').disabled=!j.running;
  const viewing=tab==='camera'&&window.uiSensor==='lidar';
  if(viewing&&j.running!==ld.running)ldView(j.running);
}
async function ldProc(action){
  const get=action==='status';
  if(!get&&!await guard('lidar',{quiet:true,title:'lidar 配信を'+(action==='start'?'起動':'停止')+'する',lines:['Jetson 上の lidar 配信プロセスを'+(action==='start'?'起動':'停止')+'します','起動・停止の結果は、画面の状態表示と点群で確かめてください']}))return;setRes('ld-res',get?'確認中…':(action==='start'?'起動中…':'停止中…'));
  ['ld-refresh','ld-start','ld-stop'].forEach(i=>$(i).disabled=true);
  try{
    const r=await fetch('/api/lidar/'+action,{method:get?'GET':'POST'}),j=await r.json();
    if(!r.ok){setRes('ld-res',j.error||('失敗 '+r.status),'bad');ldView(false);$('ld-state').textContent='状態を確認できません';$('ld-start').disabled=false}
    else{renderLidarProc(j);setRes('ld-res',j.message||'')}
  }catch(e){setRes('ld-res','取得できません: '+e,'bad')}
  $('ld-refresh').disabled=false;
}
const LS_POLL_MS=5000, LS_BADGE={1:['SAMPLING（計測中）','done'],2:['IDLE（待機・レーザー停止）','unver'],9:['READY（準備中）','plan']};
let lsBusy=false;
function lsBadge(text,cls){const b=$('ls-badge');b.textContent=text;b.className='tag '+cls}
function renderLidarSensor(j){
  $('ld-sensor-row').hidden=false;
  const[text,cls]=LS_BADGE[j.mode]||['不明（mode='+j.mode+'）','plan'];
  lsBadge(text,cls);$('ls-time').textContent=new Date().toLocaleTimeString('ja-JP')+' 確認';
  $('ls-idle').disabled=j.mode===2;$('ls-sampling').disabled=j.mode===1;
}
function lsUnknown(){lsBadge('状態を確認できません','plan');$('ls-idle').disabled=$('ls-sampling').disabled=false}
setInterval(()=>{if(!lsBusy&&!document.hidden&&tab==='camera')lsSensor('status',true)},LS_POLL_MS);  // 見ている間だけ 5 秒ごとに読み直す
async function lsSensor(action,quiet){
  const get=action==='status',idle=action==='idle';
  if(lsBusy)return;
  if(!get&&!await guard('lidar_sensor',{quiet:true,title:'LiDAR 本体を'+(idle?'停止（待機）':'起動（計測）')+'する',lines:idle
    ?['LiDAR のレーザーが止まり、点群が来なくなります','Nav2・マッピングは使えなくなります','電源を入れ直すと計測に戻ります']
    :['LiDAR が計測を再開し、点群が来ます']}))return;
  if(lsBusy)return;  // 確認ダイアログの待ち中に定期読み直しが入った場合
  lsBusy=true;
  if(!quiet){setRes('ls-res',get?'確認中…':'切り替え中…');['ls-idle','ls-sampling'].forEach(i=>$(i).disabled=true)}
  try{
    const r=await fetch(get?'/api/lidar/sensor':'/api/lidar/sensor/'+action,{method:get?'GET':'POST',cache:'no-store'});
    if(r.status===404){$('ld-sensor-row').hidden=true;setRes('ls-res','');return}  // --local でないときは出さない
    const j=await r.json();
    if(!r.ok){setRes('ls-res',j.error||('失敗 '+r.status),'bad');lsUnknown()}
    else{renderLidarSensor(j);setRes('ls-res','')}
  }catch(e){setRes('ls-res','取得できません: '+e,'bad');lsUnknown()}
  finally{lsBusy=false}
}
$('ls-idle').onclick=()=>lsSensor('idle');$('ls-sampling').onclick=()=>lsSensor('sampling');
$('ld-refresh').onclick=()=>{ldProc('status');lsSensor('status')};$('ld-start').onclick=()=>ldProc('start');$('ld-stop').onclick=()=>ldProc('stop');
$('ld-reset').onclick=()=>{Object.assign(ld,LD_VIEW0);ldDraw()};
{const c=$('ld-canvas');let drag=null;
 c.onpointerdown=e=>{drag=[e.clientX,e.clientY];c.setPointerCapture(e.pointerId);c.style.cursor='grabbing'};
 c.onpointerup=()=>{drag=null;c.style.cursor='grab'};
 c.onpointermove=e=>{if(!drag)return;ld.yaw+=(e.clientX-drag[0])*.01;ld.pitch=Math.min(Math.PI/2,Math.max(0,ld.pitch+(e.clientY-drag[1])*.01));drag=[e.clientX,e.clientY];ldDraw()};
 c.addEventListener('wheel',e=>{e.preventDefault();ld.scale=Math.min(400,Math.max(10,ld.scale*(e.deltaY<0?1.1:1/1.1)));ldDraw()},{passive:false});
 ldDraw()}
async function loadVolume(){
  setRes('vol-res','読み込み中…');
  try{
    const r=await fetch('/api/audio/volume',{cache:'no-store'}),j=await r.json();
    if(!r.ok||j.volume==null){setRes('vol-res',`取得失敗：${j.error||'code='+j.code}`,'bad');return}
    if(originalVolume==null){originalVolume=j.volume;$('vol-restore').disabled=false;$('vol-restore').textContent=`変更前（${originalVolume}）に戻す`}
    $('vol-now').textContent=`現在値 ${j.volume}`;$('vol').value=j.volume;$('vol-out').value=j.volume;
    setRes('vol-res',`取得しました（${clock(Date.now()/1000)}）`,'ok');
  }catch(e){setRes('vol-res',`サーバーに届きません（${e}）`,'bad')}
}
async function setVolume(v){
  if(originalVolume==null){setRes('vol-res','先に「読み込み」で変更前の値を取得してください','warn');return}
  setRes('vol-res','送信中…');
  try{
    const {ok,j}=await audioPost('volume',{volume:v});
    if(!ok){setRes('vol-res',`失敗：${j.error||`set_code=${j.set_code} read_code=${j.read_code}`}`,'bad');addLog(`音量 ${v} 失敗`,'bad');return}
    const match=j.volume_after===v;
    $('vol-now').textContent=`現在値 ${j.volume_after}`;
    setRes('vol-res',match?`受理 → 読み戻して ${j.volume_after} と一致（確認済み）`:`受理されたが読み戻しは ${j.volume_after}（設定値 ${v} と不一致）`,match?'ok':'bad');
    addLog(`音量 ${v} を設定（読み戻し ${j.volume_after}${match?'・一致':'・不一致'}）`,match?'':'bad');
  }catch(e){setRes('vol-res',`サーバーに届きません（${e}）`,'bad')}
}
async function setLed(r,g,b){
  if(!await guard('audio',{title:'頭部 LED の色を変える',ok:'送信する',lines:[`色 (${r}, ${g}, ${b}) を送ります。読み戻せないので、色が変わったか目で確認してください`]}))return;
  setRes('led-res','送信中…');
  try{
    const {ok,j}=await audioPost('led',{r,g,b});
    setRes('led-res',ok?`送信しました（受理 code=${j.code}）。色が変わったか目で確認してください`:`失敗：${j.error||'code='+j.code}`,ok?'warn':'bad');
    addLog(`LED (${r},${g},${b}) ${ok?'送信（受理・目視待ち）':'失敗'}`,ok?'':'bad');
  }catch(e){setRes('led-res',`サーバーに届きません（${e}）`,'bad')}
}
async function speak(){
  const text=$('tts').value.trim();if(!text){setRes('tts-res','文を入力してください','warn');return}
  if(!await guard('audio',{title:'読み上げる',ok:'読み上げる',always:true,lines:[`実機のスピーカーから「${text}」を読み上げます`,'周囲に問題ありませんか？']}))return;
  setRes('tts-res','送信中…');
  try{
    const {ok,j}=await audioPost('tts',{text,speaker_id:+$('tts-spk').value});
    setRes('tts-res',ok?`送信しました（受理 code=${j.code}）。音が出たか耳で確認してください`:`失敗：${j.error||'code='+j.code}`,ok?'warn':'bad');
    addLog(`読み上げ「${text}」${ok?'送信（受理・聴取待ち）':'失敗'}`,ok?'':'bad');
  }catch(e){setRes('tts-res',`サーバーに届きません（${e}）`,'bad')}
}
function hexRgb(h){return[1,3,5].map(i=>parseInt(h.slice(i,i+2),16))}
function initAudio(){
  $('vol').oninput=()=>{$('vol-out').value=$('vol').value};
  $('vol-load').onclick=loadVolume;
  $('vol-set').onclick=()=>setVolume(+$('vol').value);
  $('vol-restore').onclick=()=>setVolume(originalVolume);
  $('led-set').onclick=()=>setLed(...hexRgb($('led').value));
  $('led-off').onclick=()=>setLed(0,0,0);
  $('tts-go').onclick=speak;
}
function renderAudio(n){
  const live=n.jetson.st==='ok'&&n.g1.st==='ok'&&!busy;
  document.querySelectorAll('#tab-audio .arow button,#tab-audio .arow input,#tab-audio .arow select').forEach(e=>{if(e.id==='vol-restore'&&originalVolume==null)return;e.disabled=!live});
}

function render(){
  const n=nodes();
  renderChain(n);diagnose(n);
  renderMode(n);
  renderButtons(n);
  renderPending();
  /* 重い表は、取得値・状態・タブが変わったときだけ描き直す（250ms ごとの作り直しがちらつきの元だった） */
  const key=[recvAt,tab,busy,n.server.st,n.jetson.st,n.g1.st].join('|');
  if(key!==lastKey){lastKey=key;renderTelemetry(n);renderAudio(n)}
  if(window.renderMobile)window.renderMobile(n);
  if(S)$('cadence').textContent=`サーバーが約 ${S.poll_interval_s} 秒ごとに G1 を確認し、この画面が ${POLL_MS/1000} 秒ごとにサーバーへ問い合わせます。表示は最大で数秒遅れることがあります。`;
}

async function setMode(b){
  const epoch=uiRouteEpoch;
  const cur=S&&S.mode&&S.mode.state==='ok'?S.mode.fsm_id:null;
  const warn=b.danger&&(cur===500||cur===501)?'\n\n歩行中にこの操作をすると転倒する恐れがあります。':'';
  if(!await guard('mode',{title:`「${b.label}」に切り替える`,ok:'切り替える',always:true,quiet:true,danger:b.danger,
    lines:['周囲の安全を確認してください',...(warn?['歩行中にこの操作をすると転倒する恐れがあります']:[]),'物理リモコンを手元に置く']}))return;
  if(epoch!==uiRouteEpoch||tab!=='ops')return;
  busy=true;render();
  try{
    const r=await fetch('/api/mode',{method:'POST',body:JSON.stringify({id:b.id})});
    const j=await r.json();
    const resp=JSON.stringify(j);  /* G1 からの応答をそのまま履歴に残す（受理されたか、断られたかの切り分け用） */
    if(r.ok){pending={id:b.id,label:b.label,at:performance.now()};addLog(`「${b.label}」を送信（現在：${cur==null?'不明':cur}）· 応答 ${resp} · 到達待ち`,'')}
    else addLog(`「${b.label}」失敗（HTTP ${r.status}）：${j.error||'理由なし'} · 応答 ${resp}`,'bad');
  }catch(e){addLog(`「${b.label}」失敗：サーバーに届きません（${e}）`,'bad')}
  busy=false;render();
}

async function setSpeed(b){
  const epoch=uiRouteEpoch;
  if(!await guard('speed',{title:`「${b.label}」に切り替える`,ok:'切り替える',always:true,
    lines:['周囲の安全を確認してください','歩行速度が変わります。意味（低速・高速）は未確認です','物理リモコンを手元に置く']}))return;
  if(epoch!==uiRouteEpoch||tab!=='ops')return;
  busy=true;render();
  try{
    const r=await fetch('/api/speed',{method:'POST',body:JSON.stringify({value:b.value})});
    const j=await r.json(),resp=JSON.stringify(j);
    if(r.ok)addLog(`「${b.label}」を送信 · 応答 ${resp}`,'');
    else addLog(`「${b.label}」失敗（HTTP ${r.status}）：${j.error||'理由なし'} · 応答 ${resp}`,'bad');
  }catch(e){addLog(`「${b.label}」失敗：サーバーに届きません（${e}）`,'bad')}
  busy=false;render();
}

async function loadButtons(){
  const j=await (await fetch('/api/buttons',{cache:'no-store'})).json();
  const list=j.buttons;buttons=list;transitions=j.transitions||{};fallbackTo=j.fallback||[1];speedFsm=j.speed_fsm_ids||[];
  $('speed-btns').replaceChildren(...(j.speed||[]).map(b=>{
    const el=document.createElement('button');el.textContent=b.label;el.disabled=true;el.onclick=()=>setSpeed(b);return el;
  }));
  $('btns').replaceChildren(...list.map(b=>{
    const el=document.createElement('button');
    el.textContent=b.label;el.dataset.id=b.id;el.disabled=true;
    if(b.danger)el.classList.add('danger');
    el.onclick=()=>setMode(b);return el;
  }));
}

async function loadScenarios(){
  const r=await fetch('/api/scenarios',{cache:'no-store'});
  if(!r.ok)return;
  const {scenarios,current}=await r.json();
  const sel=$('scenario');
  sel.replaceChildren(...Object.entries(scenarios).map(([k,v])=>{const o=document.createElement('option');o.value=k;o.textContent=v;return o}));
  sel.value=current;
  sel.onchange=async()=>{await fetch('/api/scenario',{method:'POST',body:JSON.stringify({scenario:sel.value})});await poll(true)};
  $('mockbar').hidden=false;
}

/* 動作未確認の注意ポップアップ。実機未確認の操作は、押したときに注意を出して確認を取る。
   文面は textContent だけで描く（タスクの注意書きなどサーバー由来の文字列を HTML として解釈しない）。 */
let featureList=[];
async function featureInfo(id){
  if(!featureList.length){try{featureList=(await (await fetch('/api/features',{cache:'no-store'})).json()).features}catch(e){}}
  return featureList.find(f=>f.id===id)||null;
}
function askCaution(o){
  return new Promise(resolve=>{
    const d=$('caution'),lines=o.lines||[];
    if(d.open){resolve(false);return}
    if(typeof d.showModal!=='function'){resolve(confirm([o.title,o.unverified?'（実機未確認）':'',...lines].filter(Boolean).join('\n')));return}
    $('ca-title').textContent=o.title;
    $('ca-unver').hidden=!o.unverified;$('ca-note').textContent=o.note||'';$('ca-note').hidden=!o.note;
    $('ca-lines').replaceChildren(...lines.map(t=>{const li=document.createElement('li');li.textContent=t;return li}));
    $('ca-check').hidden=!o.needCheck;$('ca-checkbox').checked=false;
    const ok=$('ca-ok');ok.textContent=o.ok||'実行する';ok.className=o.danger?'danger':'';ok.disabled=!!o.needCheck;
    $('ca-checkbox').onchange=e=>{ok.disabled=!e.target.checked};
    d.onclose=()=>resolve(d.returnValue==='ok');
    d.returnValue='';d.showModal();
  });
}
/* 機能 id（FEATURES）が実機未確認なら注意を出す。verified なら、always のときだけ（安全確認の問い）出す。 */
async function guard(fid,o){
  const epoch=uiRouteEpoch;
  const f=await featureInfo(fid),unverified=!f||!f.verified;
  if(epoch!==uiRouteEpoch||document.hidden)return false;
  if(!unverified&&!o.always)return true;
  /* quiet: 実機確認が済んだ操作。「実機未確認」の表示は出さず、安全確認の問いだけ出す */
  const accepted=await askCaution({...o,unverified:unverified&&!o.quiet,note:o.quiet?'':f&&f.note||''});
  return accepted&&epoch===uiRouteEpoch&&!document.hidden;
}

/* 状態バッジ。未実装＝まだ無い機能、実機未確認＝実装したが G1 で試していない機能 */
const tag=(text,cls)=>{const s=document.createElement('span');s.className='tag '+cls;s.textContent=text;return s};
const statusTags=f=>!f?[]:!f.implemented?[tag('未実装','plan')]:[tag('実装済み','done'),...(f.verified?[]:[tag('実機未確認','unver')])];
const headTags=f=>!f?[]:!f.implemented?[tag('未実装','plan')]:(f.verified?[]:[tag('実機未確認','unver')]);

function renderPlanned(el,list){
  el.replaceChildren(...list.map(p=>{
    const b=document.createElement('button');
    b.disabled=true;b.title=p.note||'';b.textContent=p.label;
    const s=document.createElement('small');s.append(tag('未実装','plan'));b.append(s);return b;
  }));
}

async function loadFeatures(){
  const j=await (await fetch('/api/features',{cache:'no-store'})).json();
  document.querySelectorAll('.tags').forEach(el=>el.replaceChildren(...headTags(j.features.find(f=>f.id===el.dataset.feature))));
  renderPlanned($('planned-modes'),j.planned_modes);
  renderActions(j.actions);
  $('features').replaceChildren(...j.features.map(f=>{
    const li=document.createElement('li'),b=document.createElement('b');
    b.textContent=f.label;li.append(b,...statusTags(f));
    if(f.note){const n=document.createElement('span');n.className='note';n.textContent=f.note;li.append(n)}
    return li;
  }));
  featuresLoaded=true;
}

async function poll(once){
  try{
    const ctl=new AbortController(),t=setTimeout(()=>ctl.abort(),3000);
    const r=await fetch('/api/status',{signal:ctl.signal,cache:'no-store'});
    clearTimeout(t);
    if(!r.ok)throw new Error('HTTP '+r.status);
    const next=await r.json();
    if(!next.server||!next.jetson||!next.g1)throw new Error('状態応答の形式が不正です');
    S=next;recvAt=performance.now();fails=0;
    if(!buttons.length)await loadButtons();
    if(!featuresLoaded)await loadFeatures();
    if(!audioReady){audioReady=true;initAudio()}
    if(!volumeTried&&S.jetson.state==='ok'&&S.g1.state==='ok'){volumeTried=true;loadVolume()}
    if(S.server.mock&&$('mockbar').hidden)await loadScenarios();
  }catch(e){fails++}
  render();
  if(!once)setTimeout(poll,POLL_MS);
}

/* ---- DDS リストタブ ---- */
const DDS_LABEL={displayed:['表示済み','done'],todo:['未購読（未実装）','unver'],excluded:['対象外','plan']};
let dds=null,ddsFilter='all',tab='home';
let uiRouteEpoch=0;
function td(tr,x,cls){const c=document.createElement('td');c.textContent=x;if(cls)c.className=cls;tr.append(c);return c}

/* ---- 移動・アクション（物理リモコン代替）。送るのは 10 Hz の ctl だけ。0.5 秒途絶するとハブが止めて操作権を外す。 ---- */
const MO_MS=100;
let moEpoch=0,moClaimBusy=false;
let moSession=null,moSeq=0,moTimer=null,moBusy=0,moLim={vx:[-0.2,0.3],vy:[-0.2,0.2],wz:[-0.4,0.4]},moKeys=new Set();
function makePad(el,twoAxes){
  const knob=el.firstElementChild,v={x:0,y:0};let id=null;
  const draw=()=>{const r=el.clientWidth/2,h=el.clientHeight/2;knob.style.transform='translate('+(v.x*(r-29))+'px,'+(v.y*(h-29))+'px)'};
  const at=e=>{
    const b=el.getBoundingClientRect(),r=b.width/2,h=b.height/2;
    let x=(e.clientX-b.left-r)/(r-29),y=twoAxes?(e.clientY-b.top-h)/(h-29):0;
    const m=Math.hypot(x,y);if(m>1){x/=m;y/=m}
    v.x=Math.abs(x)<0.08?0:x;v.y=Math.abs(y)<0.08?0:y;draw();
  };
  const end=e=>{if(id===null||(e&&e.pointerId!==id))return;id=null;v.x=v.y=0;el.classList.remove('on');draw()};
  el.addEventListener('pointerdown',e=>{if(!moSession||tab!=='motion'||document.hidden||id!==null)return;id=e.pointerId;el.setPointerCapture(id);el.classList.add('on');at(e)});
  el.addEventListener('pointermove',e=>{if(e.pointerId===id)at(e)});
  ['pointerup','pointercancel','lostpointercapture'].forEach(n=>el.addEventListener(n,end));
  return {v,reset:()=>{id=null;v.x=v.y=0;el.classList.remove('on');draw()}};
}
const padMove=makePad($('pad-move'),true),padTurn=makePad($('pad-turn'),false);
const KEY_AXES={ArrowUp:['f',1],ArrowDown:['f',-1],ArrowLeft:['s',1],ArrowRight:['s',-1],q:['t',1],e:['t',-1]};
const scale=(a,[lo,hi])=>a>=0?a*hi:a*-lo;
function moTarget(){
  let f=-padMove.v.y,s=-padMove.v.x,t=-padTurn.v.x;
  moKeys.forEach(k=>{const [ax,sg]=KEY_AXES[k];if(ax==='f')f=sg;if(ax==='s')s=sg;if(ax==='t')t=sg});
  const mobile=window.mobileDriveTarget?window.mobileDriveTarget():{f:0,s:0,t:0,ratio:1};
  f=Math.max(-1,Math.min(1,f+mobile.f));s=Math.max(-1,Math.min(1,s+mobile.s));t=Math.max(-1,Math.min(1,t+mobile.t));
  return {vx:scale(f,moLim.vx)*mobile.ratio,vy:scale(s,moLim.vy)*mobile.ratio,wz:scale(t,moLim.wz)*mobile.ratio};
}
function moShow(j,note){
  const el=$('mo-state');let txt,cls='';
  if(!j){txt=note;cls='s-bad'}
  else if(moSession){txt=j.blocked?'操作中（動かせません: '+j.blocked+'）':'操作中';cls=j.blocked?'s-warn':'s-on'}
  else{
    const ev=j.last_event,why={lost:'通信が途絶えたため停止しました',estop:'緊急停止しました',disabled:'手動操作が禁止されました',sender_error:'送信に失敗したため停止しました',takeover:'他の人に操作権を取られました'}[ev&&ev.kind];
    txt=(why?why+'。':'')+(j.operator?'他の人が操作中です':(j.disable_reason?'手動操作できません: '+j.disable_reason:'操作していません'));
    cls=why?'s-warn':'';
  }
  if(note&&j)txt+=' '+note;
  el.textContent=txt;el.className=cls;
  if(j){moLim=j.limits||moLim;$('mo-lim').textContent='前後 '+moLim.vx.join('〜')+' m/s・左右 '+moLim.vy.join('〜')+' m/s・旋回 '+moLim.wz.join('〜')+' rad/s'}
  if(j?.blocked){padMove.reset();padTurn.reset();moKeys.clear();window.resetMobileDrive?.()}
  const on=!!moSession;
  $('mo-claim').disabled=on||moClaimBusy;$('mo-release').disabled=!on;$('mo-estop-float').hidden=!on;
  $('ac-claim').disabled=on||moClaimBusy;$('ac-release').disabled=!on;$('ac-state').textContent=txt;$('ac-state').className=cls;
  if(window.mobileMotionState)window.mobileMotionState(on&&!j?.blocked&&!j?.disable_reason,txt);
  document.querySelectorAll('#mo-actions button').forEach(b=>b.disabled=!on||!!(j&&j.blocked));
}
async function moPost(kind,body){
  const r=await fetch('/api/motion/'+kind,{method:'POST',body:JSON.stringify(body||{})});
  return {ok:r.ok,status:r.status,j:await r.json().catch(()=>({error:'応答を読めません'}))};
}
function moLost(j,msg){
  moSession=null;clearInterval(moTimer);moTimer=null;padMove.reset();padTurn.reset();moKeys.clear();window.resetMobileDrive?.();
  moShow(null,msg||(j&&j.error)||'操作権がありません');
}
async function moStatus(){
  if(!['motion','action'].includes(tab)||moSession)return;
  try{const r=await fetch('/api/motion/status',{cache:'no-store'});moShow(await r.json())}
  catch(e){moShow(null,'ハブに届きません')}
}
async function moTick(){
  if(!moSession||moBusy>=2)return;
  const t=moTarget(),sid=moSession;moBusy++;
  $('mo-vel').textContent='vx '+t.vx.toFixed(2)+' / vy '+t.vy.toFixed(2)+' / wz '+t.wz.toFixed(2);
  try{
    const r=await moPost('ctl',{session:sid,seq:++moSeq,...t});
    if(sid!==moSession)return;
    if(r.ok)moShow(r.j);else moLost(r.j);
  }catch(e){if(sid===moSession){moRelease();moShow(null,'ハブに届きません。操作を終了しました（ハブ側は 0.5 秒で停止します）')}}
  finally{moBusy--}
}
async function moClaim(){
  if(moClaimBusy||moSession||!['motion','action'].includes(tab)||document.hidden)return;
  const epoch=moEpoch;moClaimBusy=true;moShow(null,'操作開始の確認中…');
  try{
    if(!await guard('move',{title:'移動・アクションの操作を始める',ok:'操作を始める',danger:true,always:true,quiet:true,
      lines:['物理リモコンを手元に置く。この画面の停止は安全規格を満たさない','周囲 2 m に人・物がないことを確認する','速度上限は実機で未検証です']}))return;
    if(epoch!==moEpoch||!['motion','action'].includes(tab)||document.hidden)return;
    moShow(null,'操作権を取っています…（初回は数秒かかります）');
    const r=await moPost('claim',{});
    if(epoch!==moEpoch||document.hidden||!['motion','action'].includes(tab)){
      if(r.ok&&r.j.session)moSendRelease(r.j.session);
      return;
    }
    if(!r.ok){moShow(null,r.j.error||'操作を開始できません');return}
    moSession=r.j.session;moSeq=0;moShow(r.j);
    moTimer=setInterval(moTick,MO_MS);
    addLog('移動・アクション: 操作開始','ok');
  }catch(e){if(epoch===moEpoch)moShow(null,'ハブに届きません')}
  finally{if(epoch===moEpoch){moClaimBusy=false;$('mo-claim').disabled=!!moSession;$('ac-claim').disabled=!!moSession}}
}
function moSendRelease(sid){
  if(!(navigator.sendBeacon&&navigator.sendBeacon('/api/motion/release',JSON.stringify({session:sid}))))
    fetch('/api/motion/release',{method:'POST',body:JSON.stringify({session:sid}),keepalive:true}).catch(()=>{});
}
function moRelease(){
  moEpoch++;moClaimBusy=false;
  const sid=moSession;moLost(null,'操作を終了しました');
  if(sid)moSendRelease(sid);
}

async function moEstop(){
  moEpoch++;moClaimBusy=false;window.resetMobileDrive?.();
  moSession=null;clearInterval(moTimer);moTimer=null;padMove.reset();padTurn.reset();moKeys.clear();
  let note='緊急停止を送りました';
  try{const r=await moPost('estop');moShow(r.j);addLog('移動・アクション: 緊急停止','bad');return}
  catch(e){note='緊急停止を送れませんでした。物理リモコンで止めてください'}
  moShow(null,note);
}
async function moAction(a,b){
  if(!moSession)return;
  const session=moSession,epoch=uiRouteEpoch;
  if(!await guard('action',{title:`「${a.label}」を実行する`,lines:['SetTaskId の番号と効果は実機で確かめていない','周囲に人・物がないことを確認する']}))return;
  if(epoch!==uiRouteEpoch||session!==moSession||tab!=='action')return;
  b.disabled=true;
  try{
    const r=await moPost('action',{session:moSession,name:a.id});
    addLog('アクション: '+a.label+(r.ok?'（指令受理・動作完了は未確認）':'（失敗: '+(r.j.error||r.status)+'）'),r.ok?'ok':'bad');
    if(r.status===409&&/操作権/.test(r.j.error||''))moLost(r.j);else if(r.ok)moShow(r.j);
  }catch(e){addLog('アクション: '+a.label+'（ハブに届きません）','bad')}
  finally{if(moSession)b.disabled=false}
}
function renderActions(list){
  $('mo-actions').replaceChildren(...list.map(a=>{
    const b=document.createElement('button');b.type='button';b.disabled=true;b.textContent=a.label;b.onclick=()=>moAction(a,b);return b;
  }));
}
$('mo-claim').onclick=moClaim;
$('mo-release').onclick=moRelease;
$('ac-claim').onclick=moClaim;
$('ac-release').onclick=moRelease;
['mo-estop','mo-estop-float'].forEach(id=>$(id).addEventListener('pointerdown',e=>{e.preventDefault();moEstop()}));
window.addEventListener('keydown',e=>{
  if(!moSession||tab!=='motion'||e.repeat&&moKeys.has(e.key)||/INPUT|TEXTAREA|SELECT/.test((e.target||{}).tagName||''))return;
  const k=e.key.length===1?e.key.toLowerCase():e.key;
  if(KEY_AXES[k]){moKeys.add(k);e.preventDefault()}
});
window.addEventListener('keyup',e=>{moKeys.delete(e.key.length===1?e.key.toLowerCase():e.key)});
window.addEventListener('blur',()=>{moKeys.clear();padMove.reset();padTurn.reset();window.resetMobileDrive?.()});
document.addEventListener('visibilitychange',()=>{if(document.hidden)moRelease()});
window.addEventListener('pagehide',moRelease);
setInterval(moStatus,1000);

/* ---- タスクタブ（定義済みコマンドの実行。argv はサーバーの tasks/*.json にだけある） ---- */
let tkSel=null,tkSince=0;
const tkEls={},TK_RESULT={ok:'成功',failed:'失敗',stopped:'停止',timeout:'時間切れ'};
async function tkApi(path,body){
  const r=await fetch(path,body?{method:'POST',body:JSON.stringify(body)}:{cache:'no-store'});
  let j={};try{j=await r.json()}catch(e){}
  if(!r.ok)throw new Error(j.error||('失敗 '+r.status));
  return j;
}
function tkCard(t){
  const mk=(tag,txt,cls)=>{const e=document.createElement(tag);if(cls)e.className=cls;if(txt!=null)e.textContent=txt;return e};
  const card=mk('div',null,'tk'),st=mk('span',null,'tk-st'),row=mk('div',null,'mo-row');
  const go=mk('button','実行'),stop=mk('button','停止','danger'),lg=mk('button','ログを見る');
  go.onclick=()=>tkAct('start',t.name);stop.onclick=()=>tkAct('stop',t.name);lg.onclick=()=>tkSelect(t.name);
  row.append(go,stop,lg);
  const head=mk('b',t.title);if(!t.verified)head.append(tag('実機未確認','unver'));
  card.append(head,st,mk('span',t.description,'note'),mk('code',t.argv.join(' ')),row);
  $('tk-list').append(card);
  return tkEls[t.name]={card,st,go,stop,def:t};
}
function renderTasks(list){
  window.mobileTasks?.(list);
  list.forEach(t=>{
    const e=tkEls[t.name]||tkCard(t),run=t.state!=='idle';
    e.go.disabled=run;e.stop.disabled=t.state!=='running';
    e.st.textContent=run?(t.state==='stopping'?'停止中…':'実行中'):(t.result?`${TK_RESULT[t.result]||t.result}（終了コード ${t.exit_code}）`:'未実行');
    e.card.dataset.sel=String(t.name===tkSel);
  });
}
async function tkRefresh(){
  try{renderTasks((await tkApi('/api/tasks')).tasks);$('tk-msg').textContent=''}
  catch(e){$('tk-msg').textContent='取得できません: '+e.message;window.mobileTasks?.(null)}
  if(tab==='tasks'&&tkSel)tkLogs();
}
async function tkAct(kind,name){
  const epoch=uiRouteEpoch;
  const t=(tkEls[name]||{}).def;
  if(t&&(!t.verified||t.caution.length)){
    const run=kind==='start';
    if(!await askCaution({title:`「${t.title}」を${run?'実行':'停止'}する`,ok:run?'実行する':'停止する',danger:true,unverified:!t.verified,
      needCheck:t.caution.length>0,lines:[...t.caution,'コマンド: '+t.argv.join(' ')]}))return;
  }
  if(epoch!==uiRouteEpoch||tab!=='tasks'||document.hidden)return;
  try{await tkApi('/api/tasks/'+kind,{name});if(kind==='start')tkSelect(name)}
  catch(e){$('tk-msg').textContent=e.message}
  tkRefresh();
}
function tkSelect(name){
  tkSel=name;tkSince=0;$('tk-log').textContent='';$('tk-log-card').hidden=false;
  $('tk-log-h').textContent='ログ: '+name;tkLogs();
}
async function tkLogs(){
  const name=tkSel;if(!name)return;
  try{
    const j=await tkApi('/api/tasks/logs?name='+encodeURIComponent(name)+'&since='+tkSince);
    if(name!==tkSel)return;
    const pre=$('tk-log');
    if(j.next<tkSince){tkSince=0;pre.textContent='';return}  /* 再実行でログが作り直された */
    if(!j.lines.length)return;
    const atEnd=pre.scrollTop+pre.clientHeight>=pre.scrollHeight-8;
    pre.textContent=(j.truncated&&!pre.textContent?'…（古い行は省略）\n':'')+(pre.textContent+j.lines.map(l=>l.text+'\n').join('')).slice(-200000);
    tkSince=j.next;
    if(atEnd)pre.scrollTop=pre.scrollHeight;
  }catch(e){}
}
setInterval(tkRefresh,1000);

/* ---- マップタブ（Nav2 の地図・現在地の表示と地点の編集。Nav2 へ送る操作は無い） ---- */
const mpc=$('mp-canvas'),mpx=mpc.getContext('2d');
let mpInfo=null,mpImg=null,mpState=null,mpLocs=[],mpPick=null,mpT=null,mpP=null,mpLoadAt=0;
const MP_BRIDGE={DISCONNECTED:'未接続',STANDBY:'待機',READY:'準備完了',NAVIGATING:'走行中',FAULT:'異常',E_STOP:'非常停止',UNKNOWN:'不明'};
function mpScale(){return mpInfo?mpc.width/mpInfo.width:1}
function mpToPx(x,y){const s=mpScale();return[(x-mpInfo.origin[0])/mpInfo.resolution*s,(mpInfo.height-(y-mpInfo.origin[1])/mpInfo.resolution)*s]}
function mpToWorld(cx,cy){const s=mpScale();return[mpInfo.origin[0]+cx/s*mpInfo.resolution,mpInfo.origin[1]+(mpInfo.height-cy/s)*mpInfo.resolution]}
function mpLine(pts,color){
  if(pts.length<2)return;
  mpx.strokeStyle=color;mpx.lineWidth=2;mpx.beginPath();
  pts.forEach((p,i)=>{const[a,b]=mpToPx(p[0],p[1]);i?mpx.lineTo(a,b):mpx.moveTo(a,b)});mpx.stroke();
}
function mpDot(x,y,r,color,fill){
  const[a,b]=mpToPx(x,y);mpx.beginPath();mpx.arc(a,b,r,0,6.2832);
  if(fill){mpx.fillStyle=color;mpx.fill()}else{mpx.strokeStyle=color;mpx.lineWidth=2;mpx.stroke()}
  return[a,b];
}
function mpDraw(){
  if(!mpInfo||!mpImg){mpc.height=Math.max(120,Math.round(mpc.width*0.4));mpx.clearRect(0,0,mpc.width,mpc.height);return}
  mpc.width=Math.max(200,Math.round(mpc.clientWidth));mpc.height=Math.round(mpc.width*mpInfo.height/mpInfo.width);
  mpx.drawImage(mpImg,0,0,mpc.width,mpc.height);
  const st=mpState&&mpState.available?mpState:null;
  if(st){mpLine(st.route,'#3ec96b');mpLine(st.plan,'#4a9eff')}
  mpx.font='12px sans-serif';
  mpLocs.forEach(l=>{const[a,b]=mpDot(l.x,l.y,4,'#f2c744',true);mpx.fillStyle='#f2c744';mpx.fillText(l.name,a+6,b-6)});
  if(st&&st.goal){const[a,b]=mpToPx(st.goal.x,st.goal.y);mpx.strokeStyle='#ff9f43';mpx.lineWidth=3;mpx.beginPath();mpx.moveTo(a-7,b-7);mpx.lineTo(a+7,b+7);mpx.moveTo(a+7,b-7);mpx.lineTo(a-7,b+7);mpx.stroke()}
  if(st&&st.pose){
    const[a,b]=mpToPx(st.pose.x,st.pose.y),t=st.pose.yaw_deg*Math.PI/180,c=Math.cos(t),d=-Math.sin(t);  /* 画像の y は下向き */
    mpx.fillStyle='#ff4d4d';mpx.beginPath();
    mpx.moveTo(a+c*12,b+d*12);mpx.lineTo(a-c*7-d*6,b-d*7+c*6);mpx.lineTo(a-c*7+d*6,b-d*7-c*6);mpx.closePath();mpx.fill();
  }
  if(mpPick)mpDot(mpPick.x,mpPick.y,7,'#ff3b30',false);
}
function mpText(){
  const st=mpState,ex=$('mp-excl');
  if(!st){return}
  ex.className='res '+(st.exclusive_reason?'warn':'');
  ex.textContent=st.exclusive_reason?('手動操作は無効: '+st.exclusive_reason):'手動操作の排他: なし（Nav2 の稼働を検出していません）';
  const hb=st.heartbeat||{};
  const parts=[];
  if(st.available){
    parts.push('Nav2: '+(st.connected?'稼働中':'未接続'),'ブリッジ: '+(MP_BRIDGE[st.bridge_state]||st.bridge_state),
      '巡回: '+st.patrol.state+(st.patrol.total?` (${st.patrol.index}/${st.patrol.total})`:''));
    if(st.pose)parts.push(`現在地 x=${f(st.pose.x,2)} y=${f(st.pose.y,2)} 向き=${f(st.pose.yaw_deg,0)}°`);else parts.push('現在地: 未取得');
    if(st.message)parts.push(st.message);
    $('mp-msg').textContent='';
  }else $('mp-msg').textContent='アダプタから読めません: '+(st.reason||'不明');
  parts.push(hb.enabled?`生存信号: ${hb.present?'送信中':'停止中'}（累計 ${hb.sent}）`:'生存信号: 無効（--heartbeat-to 未設定）');
  $('mp-info').textContent=parts.join(' ／ ');
}
async function mpLoadMap(){
  mpLoadAt=Date.now();
  try{
    const info=await tkApi('/api/nav/map.json');
    const r=await fetch('/api/nav/map.png',{cache:'no-store'});if(!r.ok)throw new Error('地図を取得できません '+r.status);
    const url=URL.createObjectURL(await r.blob()),img=new Image();
    img.onload=()=>{URL.revokeObjectURL(url);mpInfo=info;mpImg=img;mpDraw()};
    img.onerror=()=>URL.revokeObjectURL(url);
    img.src=url;
  }catch(e){$('mp-msg').textContent='地図: '+e.message}
}
async function mpLocations(){
  try{mpLocs=(await tkApi('/api/locations')).locations}catch(e){$('mp-res').textContent='地点を取得できません: '+e.message;return}
  const box=$('mp-locs');box.textContent='';
  if(!mpLocs.length){const p=document.createElement('p');p.className='note';p.textContent='登録済みの地点はありません';box.append(p)}
  mpLocs.forEach(l=>{
    const row=document.createElement('div'),nm=document.createElement('b'),pos=document.createElement('span'),pk=document.createElement('button'),del=document.createElement('button');
    row.className='mp-loc';nm.textContent=l.name;pos.className='note';pos.textContent=`x=${f(l.x,2)} y=${f(l.y,2)} 向き=${f(l.yaw_deg,0)}°`;
    pk.type=del.type='button';pk.textContent='編集に入れる';del.textContent='削除';del.className='danger';
    pk.onclick=()=>{$('mp-name').value=l.name;$('mp-x').value=l.x;$('mp-y').value=l.y;$('mp-yaw').value=l.yaw_deg;mpPick={x:l.x,y:l.y};mpDraw()};
    del.onclick=()=>mpDelete(l.name);
    row.append(nm,pos,pk,del);box.append(row);
  });
  mpDraw();
}
async function mpTick(){
  if(tab!=='map')return;
  try{mpState=await tkApi('/api/nav/state')}catch(e){mpState={available:false,reason:e.message,heartbeat:{}}}
  mpText();
  if(mpState.available&&!mpImg&&Date.now()-mpLoadAt>5000)mpLoadMap();
  mpDraw();
}
function mpPresence(){
  if(tab!=='map'||document.hidden)return;
  fetch('/api/nav/presence',{method:'POST',body:'{}'}).catch(()=>{});
}
function mpStart(){
  if(mpT)return;
  mpLocations();mpTick();mpT=setInterval(mpTick,1000);mpP=setInterval(mpPresence,500);mpPresence();
}
function mpStop(){clearInterval(mpT);clearInterval(mpP);mpT=mpP=null}
mpc.onclick=ev=>{
  if(!mpInfo)return;
  const r=mpc.getBoundingClientRect(),[x,y]=mpToWorld((ev.clientX-r.left)*mpc.width/r.width,(ev.clientY-r.top)*mpc.height/r.height);
  mpPick={x,y};$('mp-x').value=x.toFixed(3);$('mp-y').value=y.toFixed(3);if(!$('mp-yaw').value)$('mp-yaw').value='0';mpDraw();
};
$('mp-cur').onclick=()=>{
  const p=mpState&&mpState.available&&mpState.pose;
  if(!p){$('mp-res').className='res warn';$('mp-res').textContent='現在地を取得できていません';return}
  $('mp-x').value=p.x.toFixed(3);$('mp-y').value=p.y.toFixed(3);$('mp-yaw').value=Math.max(-180,Math.min(180,p.yaw_deg)).toFixed(1);mpPick={x:p.x,y:p.y};mpDraw();
};
$('mp-save').onclick=async()=>{
  const res=$('mp-res'),num=id=>$(id).value.trim()===''?NaN:Number($(id).value);
  const body={name:$('mp-name').value.trim(),x:num('mp-x'),y:num('mp-y'),yaw_deg:num('mp-yaw')};
  if(!body.name||[body.x,body.y,body.yaw_deg].some(v=>!Number.isFinite(v))){res.className='res bad';res.textContent='名前・x・y・向きを数値で入れてください';return}
  try{const j=await tkApi('/api/locations/save',body);res.className='res ok';res.textContent='登録しました: '+j.name;mpLocations()}
  catch(e){res.className='res bad';res.textContent=e.message}
};
async function mpDelete(name){
  const res=$('mp-res');
  try{await tkApi('/api/locations/delete',{name});res.className='res ok';res.textContent='削除しました: '+name;mpLocations()}
  catch(e){res.className='res bad';res.textContent=e.message}
}
document.addEventListener('visibilitychange',()=>{if(!document.hidden&&tab==='map')mpPresence()});
window.addEventListener('resize',()=>{if(tab==='map')mpDraw()});

/* ---- トピック通信（DDS タブ内）。DDS から来た値は textContent だけで描く ---- */
let tpT=null,tpInfo=null;
function tpMsg(cls,text){const e=$('tp-msg');e.className='res '+cls;e.textContent=text}
async function tpLoad(){
  try{tpInfo=await tkApi('/api/topics')}catch(e){tpInfo=null;$('tp-card').hidden=/404|not found/i.test(e.message);tpMsg('warn','トピック通信は使えません: '+e.message);return}
  $('tp-card').hidden=false;tpMsg('','');
  const sel=$('tp-type');if(!sel.children.length)tpInfo.supported_types.forEach(t=>{const o=document.createElement('option');o.value=o.textContent=t;sel.append(o)});
  const box=$('tp-pub');box.textContent='';
  if(!tpInfo.publish.length){const p=document.createElement('p');p.className='note';p.textContent='送信の許可リストは空です（何も送れません）';box.append(p)}
  tpInfo.publish.forEach(r=>{
    const row=document.createElement('div'),nm=document.createElement('b'),inp=document.createElement('input'),btn=document.createElement('button');
    row.className='arow';nm.textContent=r.topic;inp.type='text';inp.maxLength=r.max_bytes;inp.placeholder=`文字列（${r.max_bytes} バイトまで・${r.min_interval_s} 秒間隔）`;
    btn.type='button';btn.textContent='送信';
    btn.onclick=async()=>{
      if(!await guard('topics',{title:'トピックを送信する',ok:'送信する',lines:[`${r.topic} へ「${inp.value}」（${new TextEncoder().encode(inp.value).length} バイト）を送ります`]}))return;
      try{const j=await tkApi('/api/topics/publish',{topic:r.topic,data:inp.value});tpMsg('ok',`送信しました: ${j.published}（${j.bytes} バイト）`)}
      catch(e){tpMsg('bad',e.message)}
    };
    row.append(nm,inp,btn);box.append(row);
  });
}
async function tpTick(){
  if(tab!=='dds'||!tpInfo||!tpInfo.subscribed.length&&!$('tp-table').querySelector('tbody').children.length)return;
  let items;try{items=(await tkApi('/api/topics/received')).items}catch(e){tpMsg('bad',e.message);return}
  const tb=$('tp-table').querySelector('tbody');tb.textContent='';
  items.forEach(it=>{
    const tr=document.createElement('tr');
    td(tr,it.topic);td(tr,it.age_s==null?'受信なし':`${f(it.age_s,1)} 秒前（${it.count} 件）`,it.age_s==null?'live-none':'live-ok');
    const m=td(tr,'');const pre=document.createElement('code');pre.textContent=it.message==null?'—':JSON.stringify(it.message).slice(0,300);m.append(pre);
    const c=td(tr,'');const b=document.createElement('button');b.type='button';b.textContent='解除';b.className='sm';
    b.onclick=async()=>{try{await tkApi('/api/topics/unsubscribe',{topic:it.topic});await tpLoad();tpTick()}catch(e){tpMsg('bad',e.message)}};
    c.append(b);tb.append(tr);
  });
}
$('tp-sub').onclick=async()=>{
  try{const j=await tkApi('/api/topics/subscribe',{topic:$('tp-topic').value.trim(),type:$('tp-type').value});tpMsg('ok','購読しました: '+j.topic);await tpLoad();tpTick()}
  catch(e){tpMsg('bad',e.message)}
};
function tpStart(){tpLoad().then(tpTick);if(!tpT)tpT=setInterval(tpTick,2000)}
function tpStop(){clearInterval(tpT);tpT=null}

/* 主ナビ 5 つ（操作／状態／マップ・タスク／センサ／開発）。画面が無いグループは出さない。
   タブ名 = #hash = API 名は従来どおり。画面を足すときは該当グループの配列に入れる。 */
const GROUPS=[['home','ホーム',['home']],['ops','操作',['ops','motion','action','audio']],['state','状態',['state','joints']],['maptask','マップ・タスク',['map','tasks']],['sensor','センサー',['camera']],['dev','開発',['dds','settings']]];
const TABS=GROUPS.flatMap(g=>g[2]);
const ROUTES={home:'home',mode:'ops',remote:'motion',actions:'action',status:'state',voice:'audio',sensors:'camera',joints:'joints',map:'map',tasks:'tasks',dds:'dds',settings:'settings'};
const HASHES=Object.fromEntries(Object.entries(ROUTES).map(([k,v])=>[v,k]));
const TITLES={home:'G1 Control',ops:'モード切り替え',motion:'リモコン移動',action:'アクション',audio:'音声',state:'状態詳細',joints:'関節詳細',map:'マップ',tasks:'タスク',camera:'センサー',dds:'DDS',settings:'設定'};
function setTab(request,fromHistory=false){
  let t=ROUTES[request]||request;if(!TABS.includes(t))t='home';
  if(window.uiRouteReady&&t===tab)return;
  uiRouteEpoch++;
  if(t!==tab||!['motion','action'].includes(t))moRelease();
  if($('caution').open){$('caution').returnValue='cancel';$('caution').close()}
  tab=t;
  TABS.forEach(k=>{$('tab-'+k).hidden=k!==t;$('tab-b-'+k).setAttribute('aria-pressed',String(k===t));$('tab-b-'+k).hidden=false});
  $('ui-title').textContent=TITLES[t];document.title=TITLES[t]+' · G1 Control';
  $('ui-back').setAttribute('aria-label',t==='home'?'メニューを開く':t==='joints'?'状態詳細へ戻る':'ホームへ戻る');
  $('ui-back').querySelector('use').setAttribute('href',t==='home'?'#i-menu':'#i-back');
  $('ui-navigation').close();
  stopCameras();ldView(false);
  if(t==='camera')window.openSensorView?.();
  if(!fromHistory&&location.hash!=='#'+HASHES[t])history.pushState(null,'','#'+HASHES[t]);
  if(t==='dds'){if(!dds)loadDds();else renderDds()}
  t==='dds'?tpStart():tpStop();
  if(t==='settings')loadSettings();
  if(t==='motion'||t==='action')moStatus();
  if(t==='tasks')tkRefresh();
  t==='map'?mpStart():mpStop();
  lastKey='';window.uiRouteReady=true;
  window.scrollTo(0,0);$('tab-'+t).focus({preventScroll:true});
  render();
}

async function loadDds(){
  try{dds=await(await fetch('/api/dds',{cache:'no-store'})).json()}catch(e){$('dds-sum').textContent='一覧を取得できません';return}
  renderDds();
}
function buildDdsFilters(){
  [['all','すべて'],['displayed','表示済み'],['todo','未購読（未実装）'],['excluded','対象外']].forEach(([k,l])=>{
    const b=document.createElement('button');b.className='sm';b.textContent=l;b.dataset.k=k;
    b.onclick=()=>{ddsFilter=k;renderDds()};$('dds-filters').append(b);
  });
}
function renderDds(){
  if(!dds||tab!=='dds')return;
  const cnt={displayed:0,todo:0,excluded:0};dds.topics.forEach(t=>cnt[t.status]++);
  $('dds-sum').textContent=`全 ${dds.topics.length} 件：表示済み ${cnt.displayed} ／ 未購読（未実装）${cnt.todo} ／ 対象外 ${cnt.excluded}`;
  if(!$('dds-filters').children.length)buildDdsFilters();
  [...$('dds-filters').children].forEach(b=>b.setAttribute('aria-pressed',b.dataset.k===ddsFilter));
  const n=S?nodes():null,T=S&&S.telemetry,live=!!T&&!!n&&n.jetson.st==='ok'&&n.g1.st==='ok';
  const tb=document.querySelector('#dds-table tbody');tb.textContent='';
  const visible=dds.topics.filter(t=>(ddsFilter==='all'||t.status===ddsFilter)&&(!window.ddsSearchMatches||window.ddsSearchMatches(t)));
  $('ui-dds-empty').hidden=visible.length>0;
  visible.forEach(t=>{
    const tr=document.createElement('tr');
    const open=document.createElement('button');open.type='button';open.className='topic-open';open.textContent=t.topic;open.onclick=()=>window.selectDdsTopic?.(t);
    td(tr,'').append(open);td(tr,t.type);
    const c=td(tr,'');const tg=document.createElement('span');tg.className='tag '+DDS_LABEL[t.status][1];tg.style.marginLeft='0';tg.textContent=DDS_LABEL[t.status][0];c.append(tg);
    let rx='—',cls='';
    if(t.status==='displayed'){
      const a=live&&T.ages?T.ages[t.topic]:undefined;
      if(a==null){rx=live?'受信なし':'未確認';cls='live-none'}else{rx=`${f(a,1)} 秒前`;cls=a<=5?'live-ok':'live-none'}
    }
    td(tr,rx,cls);td(tr,t.note);tb.append(tr);
  });
  const sb=document.querySelector('#svc-table tbody');sb.textContent='';
  dds.services.forEach(v=>{const tr=document.createElement('tr');td(tr,v.name);td(tr,v.status,v.status==='使用中'?'live-ok':(v.status==='SDK あり・未実装'?'':'live-none'));td(tr,v.note);sb.append(tr)});
}
/* ---- 設定タブ / 自動確認の停止 ---- */
const form=$('set-form');
async function loadSettings(){
  try{
    const j=await(await fetch('/api/settings',{cache:'no-store'})).json();
    for(const [k,v] of Object.entries(j.settings))if(form.elements[k])form.elements[k].value=v;
    $('set-res').textContent='';
  }catch(e){$('set-res').textContent='設定を読めません（サーバーに届きません）'}
}
form.onsubmit=async ev=>{
  ev.preventDefault();
  const body=Object.fromEntries(new FormData(form).entries()),res=$('set-res');
  res.className='res';res.textContent='保存中…';
  try{
    const r=await fetch('/api/settings',{method:'POST',body:JSON.stringify(body)}),j=await r.json();
    if(!r.ok){res.className='res bad';res.textContent=j.error||('失敗 '+r.status);return}
    for(const [k,v] of Object.entries(j.settings))if(form.elements[k])form.elements[k].value=v;
    res.className='res ok';res.textContent='保存しました。Jetson への接続を張り直します。';
    poll(true);
  }catch(e){res.className='res bad';res.textContent='サーバーに届きません（'+e+'）'}
};
async function monitorCtl(body){
  try{await fetch('/api/monitor',{method:'POST',body:JSON.stringify(body)})}catch(e){}
  await poll(true);
}
$('mon-toggle').onclick=()=>monitorCtl({paused:!(S&&S.paused)});
$('mon-once').onclick=()=>monitorCtl({check:true});
$('hub-stop').onclick=async()=>{
  if(!await guard('hubstop',{title:'ハブを停止する',ok:'停止する',danger:true,always:true,needCheck:true,
    lines:['このコンソール自体が止まり、この画面から再起動できなくなります','再起動は Jetson で行います（自動起動に設定していれば、Jetson の再起動で戻ります）','カメラ・lidar の配信プロセスは別のため止まらない見込みです（未確認）。必要なら先に停止してください']}))return;
  setRes('hub-res','停止中…');
  try{
    const r=await fetch('/api/hub/stop',{method:'POST',body:JSON.stringify({confirm:true})}),j=await r.json();
    if(!r.ok){setRes('hub-res',j.error||('失敗 '+r.status),'bad');return}
    setRes('hub-res',j.message,'ok');$('hub-stop').disabled=true;
  }catch(e){setRes('hub-res','サーバーに届きません（'+e+'）','bad')}
};
TABS.forEach(k=>{$('tab-b-'+k).onclick=()=>setTab(HASHES[k])});
GROUPS.forEach(([key,label,tabs])=>{
  const group=document.createElement('section'),heading=document.createElement('h3');heading.textContent=label;group.append(heading);
  tabs.forEach(t=>{const button=$('tab-b-'+t);button.removeAttribute('role');button.removeAttribute('aria-selected');button.setAttribute('aria-label',TITLES[t]);group.append(button)});
  $('nav-main').append(group);
});
$('nav-sub').hidden=true;
window.addEventListener('popstate',()=>setTab(location.hash.slice(1),true));
window.addEventListener('hashchange',()=>setTab(location.hash.slice(1),true));
// mobile.js finishes registration before choosing the first full screen.
setInterval(render,250);setInterval(renderDds,1000);
poll();
