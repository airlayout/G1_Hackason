/* スマホ用の表示・入力。運動指令と許可条件は既存のコントローラを使う。 */
(function(root){
  'use strict';
  const axes={forward:[1,0,0],backward:[-1,0,0],left:[0,1,0],right:[0,-1,0],'turn-left':[0,0,1],'turn-right':[0,0,-1]};
  class DriveInput{
    constructor(){this.held=new Map();this.ratio=0.4}
    press(token,name){if(axes[name])this.held.set(token,name)}
    release(token){this.held.delete(token)}
    reset(){this.held.clear()}
    target(){const sum=[0,0,0];for(const name of this.held.values())axes[name].forEach((v,i)=>sum[i]+=v);const clamp=v=>Math.max(-1,Math.min(1,v));return{f:clamp(sum[0]),s:clamp(sum[1]),t:clamp(sum[2]),ratio:this.ratio}}
  }
  function summary(snapshot,age,unreachable=false){
    const fresh=!!snapshot&&!unreachable&&!snapshot.paused&&age!=null&&age<=8&&snapshot.jetson?.state==='ok'&&snapshot.g1?.state==='ok';
    const battery=snapshot?.telemetry?.battery;
    const soc=fresh&&battery&&Number.isFinite(battery.soc)&&battery.soc>=0&&battery.soc<=100&&battery.age_s!=null&&battery.age_s+age<=8?battery.soc:null;
    const mode=fresh?(snapshot.mode?.label||'不明'):'未確認';
    const connection=unreachable?'サーバー未接続':snapshot?.paused?'確認停止中':!snapshot?'確認中':age==null||age>8?'更新停止':fresh?'接続中':'機体未接続';
    return{fresh,soc,mode,connection};
  }
  function jointIndexes(group){return group==='legs'?[0,1,2,3,4,5,6,7,8,9,10,11]:group==='waist'?[12,13,14]:[15,16,17,18,19,20,21,22,23,24,25,26,27,28]}
  const api={DriveInput,summary,jointIndexes};
  if(typeof module!=='undefined')module.exports=api;
  root.MobileUI=api;
  if(typeof document==='undefined')return;
  const by=id=>document.getElementById(id);
  const text=(id,value)=>{const e=by(id);if(e.textContent!==String(value))e.textContent=value};
  let preferences={name:'マイ G1',compact:false};
  try{const saved=JSON.parse(localStorage.getItem('g1-control.display.v1'));if(saved&&typeof saved.name==='string')preferences={name:saved.name.trim().slice(0,20)||'マイ G1',compact:saved.compact===true}}catch(e){}
  function showPreferences(){document.body.classList.toggle('compact-lists',preferences.compact);text('ui-subtitle',tab==='home'?preferences.name:'G1 CONTROL')}
  by('ui-robot-name').value=preferences.name;by('ui-compact').checked=preferences.compact;
  by('ui-preferences').onsubmit=e=>{e.preventDefault();const name=by('ui-robot-name').value.trim();if(!name){text('ui-pref-result','表示名を入力してください。');return}preferences={name:name.slice(0,20),compact:by('ui-compact').checked};let stored=true;try{localStorage.setItem('g1-control.display.v1',JSON.stringify(preferences))}catch(e){stored=false}showPreferences();text('ui-pref-result',stored?'このブラウザに保存しました。':'このタブに反映しました。ブラウザには保存できませんでした。')};
  function openMenu(){moRelease();by('ui-navigation').showModal()}
  by('ui-menu').onclick=openMenu;
  by('ui-back').onclick=()=>tab==='home'?openMenu():setTab(tab==='joints'?'status':'home');
  by('ui-menu-close').onclick=()=>by('ui-navigation').close();
  by('ui-navigation').addEventListener('click',e=>{if(e.target===by('ui-navigation'))by('ui-navigation').close()});
  document.querySelectorAll('a[href^="#"]').forEach(a=>a.addEventListener('click',e=>{e.preventDefault();setTab(a.hash.slice(1))}));
  root.mobileTasks=list=>{if(!list){const banner=by('ui-task-banner');banner.hidden=false;banner.textContent='タスク状態を取得できません →';return}const running=list.filter(t=>t.state!=='idle');const banner=by('ui-task-banner');banner.hidden=!running.length;if(running.length)banner.textContent=`${running[0].title} · ${running[0].state==='stopping'?'停止中':'実行中'} →`};
  const drive=new DriveInput();let driveAllowed=false;
  root.mobileDriveTarget=()=>drive.target();
  const driveButtons=[...document.querySelectorAll('[data-drive]')];
  root.resetMobileDrive=()=>{drive.reset();driveButtons.forEach(b=>b.classList.remove('is-held'))};
  root.mobileMotionState=(allowed,message)=>{driveAllowed=allowed;if(!allowed)root.resetMobileDrive();driveButtons.forEach(b=>b.disabled=!allowed);text('ui-drive-state',message||'操作を開始すると方向ボタンが使えます。')};
  function press(button,token){if(!driveAllowed||tab!=='motion'||document.hidden)return false;drive.press(token,button.dataset.drive);button.classList.add('is-held');moTick();return true}
  function release(button,token){drive.release(token);button.classList.remove('is-held');if(moSession)moTick()}
  driveButtons.forEach(button=>{
    button.addEventListener('pointerdown',e=>{if(e.button!==0)return;e.preventDefault();if(press(button,e.pointerId))button.setPointerCapture(e.pointerId)});
    ['pointerup','pointercancel','lostpointercapture'].forEach(name=>button.addEventListener(name,e=>release(button,e.pointerId)));
    button.addEventListener('contextmenu',e=>e.preventDefault());
    button.addEventListener('keydown',e=>{if((e.key===' '||e.key==='Enter')&&!e.repeat){e.preventDefault();press(button,'key:'+button.dataset.drive)}});
    button.addEventListener('keyup',e=>{if(e.key===' '||e.key==='Enter'){e.preventDefault();release(button,'key:'+button.dataset.drive)}});
    button.addEventListener('blur',()=>release(button,'key:'+button.dataset.drive));
  });
  document.querySelectorAll('[data-drive-speed]').forEach(button=>button.onclick=()=>{root.resetMobileDrive();padMove.reset();padTurn.reset();moKeys.clear();drive.ratio=Number(button.dataset.driveSpeed);document.querySelectorAll('[data-drive-speed]').forEach(b=>b.setAttribute('aria-pressed',String(b===button)));if(moSession)moTick()});
  let group='legs',selectedJoint=3,lastJointData='',jointButtons=new Map();
  function drawJointList(){const list=by('ui-joint-list');list.textContent='';jointButtons=new Map();jointIndexes(group).forEach(index=>{const button=document.createElement('button'),name=document.createElement('strong'),value=document.createElement('span');button.type='button';button.dataset.jointIndex=index;name.textContent=JOINT_NAMES[index]||'#'+index;button.append(name,value);button.onclick=()=>{selectedJoint=index;lastJointData='';renderJointDetail()};list.append(button);jointButtons.set(index,{button,value})})}
  function renderJointDetail(){
    const n=nodes(),data=S?.telemetry?.joints,live=summary(S,ageNow(),fails>=2).fresh&&data?.age_s!=null&&data.age_s+ageNow()<=3;
    const key=[recvAt,live,selectedJoint,group].join('|');if(key===lastJointData)return;lastJointData=key;
    for(const [index,row]of jointButtons){const m=live?data.items[index]:null;row.value.textContent=m?`${f(m.q*RAD,1)}° · ${f(Math.max(...m.temperature),0)}℃`:'未取得';row.button.setAttribute('aria-pressed',String(index===selectedJoint))}
    const m=live?data.items[selectedJoint]:null,c=live?S.telemetry?.lowcmd?.items[selectedJoint]:null;
    setKV('ui-joint-detail',[
      ['選択中',JOINT_NAMES[selectedJoint]||'#'+selectedJoint],['角度',m?f(m.q*RAD,1)+'°':'—'],['速度 [rad/s]',m?f(m.dq):'—'],['トルク推定',m?f(m.tau):'—'],['温度',m?f(Math.max(...m.temperature),0)+'℃':'—'],['状態 mode',m?.mode??'—'],['欠落状態',m?.lost??'—'],['指令角度',c?f(c.q*RAD,1)+'°':'—'],['kp / kd',c?f(c.kp,0)+' / '+f(c.kd,1):'—']],true);
  }
  document.querySelectorAll('[data-joint-group]').forEach(button=>button.onclick=()=>{group=button.dataset.jointGroup;selectedJoint=jointIndexes(group)[0];document.querySelectorAll('[data-joint-group]').forEach(b=>b.setAttribute('aria-pressed',String(b===button)));lastJointData='';drawJointList();renderJointDetail()});
  root.uiSensor='camera';
  root.openSensorView=()=>{
    stopCameras();ldView(false);
    document.querySelectorAll('[data-sensor-panel]').forEach(panel=>panel.hidden=panel.dataset.sensorPanel!==root.uiSensor);
    if(tab!=='camera'||document.hidden)return;
    if(root.uiSensor==='camera'){camProc('status');loadCameras()}
    if(root.uiSensor==='lidar'){ldProc('status');lsSensor('status')}
  };
  document.querySelectorAll('[data-sensor]').forEach(button=>button.onclick=()=>{root.uiSensor=button.dataset.sensor;uiRouteEpoch++;document.querySelectorAll('[data-sensor]').forEach(b=>b.setAttribute('aria-pressed',String(b===button)));root.openSensorView()});
  document.addEventListener('visibilitychange',()=>{if(document.hidden){stopCameras();ldView(false)}else if(tab==='camera')root.openSensorView()});
  root.ddsSearchMatches=topic=>{const query=by('ui-dds-search').value.trim().toLowerCase();return[topic.topic,topic.type,topic.note].join(' ').toLowerCase().includes(query)};
  by('ui-dds-search').oninput=()=>{by('ui-dds-selected').hidden=true;renderDds()};
  root.selectDdsTopic=topic=>{const el=by('ui-dds-selected');el.hidden=false;el.textContent=JSON.stringify(topic,null,2)};
  root.renderMobile=n=>{
    const age=ageNow(),state=summary(S,age,fails>=2);
    showPreferences();
    text('ui-connection',state.connection);by('ui-connection').className=state.fresh?'connected':'unavailable';
    text('home-connection',state.connection);text('ui-mode',state.mode);text('home-mode',state.mode);
    text('ops-mode',state.mode);text('ops-mode-sub',state.fresh&&S?.mode?.fsm_id!=null?'FSM ID '+S.mode.fsm_id:'');
    text('ui-battery',state.soc==null?'—':state.soc+'%');text('home-battery',state.soc==null?'—':state.soc+'%');
    by('home-battery-bar').style.width=state.soc==null?'0%':state.soc+'%';
    text('home-freshness',age==null?'状態を取得しています。':`最後の確認 ${f(age,1)} 秒前${S?.paused?' · 自動確認を停止中':''}`);
    const notice=by('ui-notice');notice.hidden=state.fresh&&(state.soc==null||state.soc>20)&&S?.mode?.state!=='debug';
    text('ui-notice',!state.fresh?state.connection+' · 状態詳細で確認してください。':S?.mode?.state==='debug'?'デバッグモードです。モード切り替えは無効です。':state.soc!=null&&state.soc<=20?'バッテリー残量が少なくなっています。':'');
    by('environment-label').hidden=!S?.server?.mock;
    if(tab==='joints')renderJointDetail();
    if(tab==='camera'&&root.uiSensor==='imu'){
      const imu=state.fresh?S?.telemetry?.imu:null;
      setKV('ui-imu-values',[['ロール / ピッチ / ヨー',imu?.rpy?.map(v=>f(v*RAD,1)+'°').join(' / ')||'—'],['角速度 [rad/s]',imu?.gyro?.map(v=>f(v)).join(' / ')||'—'],['加速度 [m/s²]',imu?.accel?.map(v=>f(v)).join(' / ')||'—'],['温度',imu?.temperature!=null?imu.temperature+'℃':'—']],true);
    }
  };
  drawJointList();
  setTab(location.hash.slice(1)||'home',true);
  if(!location.hash)history.replaceState(null,'','#home');
})(typeof window==='undefined'?globalThis:window);
