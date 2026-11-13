import {test} from 'node:test';
import assert from 'node:assert/strict';
import net from 'node:net';
import {createHash} from 'node:crypto';
import {readFile} from 'node:fs/promises';
import {defaults,validate,validateParallel,parseIni,formatIni} from './config.mjs';
import {Device,readiness,floatWord,matchingFirmware} from './device.mjs';
import {Rpc} from './rpc.mjs';
test('INI roundtrip, duplicate/unknown/nonfinite/bounds and ramp guards',()=>{
 assert.deepEqual(parseIni(formatIni(defaults)),defaults);
 for(const input of [{...defaults,Kp:NaN},{...defaults,BackwardKp:NaN},{...defaults,BackwardKp:20.01},{...defaults,BackwardLimitPercent:101},{...defaults,Kd:Infinity},{...defaults,OutputLimitPercent:101},{...defaults,RunMs:30001},{...defaults,RightBoostPercent:1.8},{...defaults,RunMs:100,RampStepMs:50},{...defaults,OutputLimitPercent:1,BackwardLimitPercent:35,RunMs:200,RampStepMs:8},{...defaults,GyroPredictMs:29},{...defaults,GyroPredictMs:301},{...defaults,Kp:20.01},{...defaults,Kd:2.01},{...defaults,RightBoostPercent:10.25},{...defaults,RampStepMs:201},{...defaults,BoostMaxBasePercent:61},{...defaults,SpeedKp:NaN},{...defaults,SpeedKp:20.01},{...defaults,SpeedKi:10.01},{...defaults,SpeedTiltLimitDeg:5.01},{...defaults,SpeedTiltLimitDeg:-0.1},{...defaults,other:1}])assert.throws(()=>validate(input));
 assert.throws(()=>parseIni(formatIni(defaults)+'Kp=2\n'));
 assert.throws(()=>parseIni(formatIni(defaults).replace('Kp=4','Kp=oops')));
 assert.equal(validate({...defaults,RampStepMs:0}).RampStepMs,0);
 assert.doesNotThrow(()=>validate({...defaults,Kp:20,BackwardKp:20,Kd:2,OutputLimitPercent:60,BackwardLimitPercent:60,RunMs:30000,RampStepMs:200,RightBoostPercent:10,BoostMaxBasePercent:60,GyroPredictMs:300}));
 for(const v of [-0.25,6.25,0.1,NaN])assert.throws(()=>validate({...defaults,CommonBoostPercent:v}));
 assert.equal(parseIni(formatIni(defaults).replace('CommonBoostPercent=2\n','')).CommonBoostPercent,2);
 assert.equal(validate({...defaults,CommonBoostPercent:0}).CommonBoostPercent,0);
 for(const change of [{TaskSpeedCmS:0},{TaskSpeedCmS:10.1},{TaskSpeedCmS:NaN},{TaskRampMs:299},{TaskRampMs:2001},{TaskRampMs:1000.5}])assert.throws(()=>validate({...defaults,...change}));
 const old=parseIni(formatIni(defaults).replace('TaskSpeedCmS=3\n','').replace('TaskRampMs=1000\n',''));
 assert.equal(old.TaskSpeedCmS,3);assert.equal(old.TaskRampMs,1000);
});
const ready={pd_window_active:0,pd_window_latched:0,pd_window_fault:0,motor_guard_state:1,imu_fusion_mode:1,imu_valid:1,imu_fusion_valid:1,uptime_ms:200,imu_last_sample_ms:195,imu_body_pitch_deg:-1,imu_pitch_rate_dps:1,pd_window_run_count:2,tune_result_id:10};
test('start requires valid pose and tilt, accepts gyro mode and old unlatched fault',()=>{
 assert.doesNotThrow(()=>readiness(ready));
 assert.doesNotThrow(()=>readiness({...ready,imu_fusion_mode:2,pd_window_fault:14,imu_pitch_rate_dps:30,imu_body_pitch_deg:14.9}));
 for(const changed of [{pd_window_active:1},{pd_window_latched:1},{imu_valid:0},{imu_fusion_mode:0},{imu_last_sample_ms:100},{imu_body_pitch_deg:NaN},{imu_body_pitch_deg:-15.1},{imu_pitch_rate_dps:NaN}])assert.throws(()=>readiness({...ready,...changed}));
});
function mock(){const d=new Device({});d.connected=true;d.writes=[];d.snapshot=async()=>({...ready});d.put=async(n,v)=>{d.writes.push([n,v])};return d}
test('arming delegates preparation once to MCU; stop has no latent host timer',async()=>{
 const d=mock();d.snapshot=async()=>d.writes.length?{...ready,local_state:1,local_remaining_ms:2900}:ready;await d.arm();assert.deepEqual(d.writes,[['local_alive_counter',1],['local_command',1]]);
 d.stopNow=async()=>{d.writes.push(['stop',9])};await d.stop();await d.tick();assert.equal(d.countdownUntil,0);assert.deepEqual(d.writes,[['local_alive_counter',1],['local_command',1],['stop',9]]);
});
test('apply writes staging before commit, never sends motor command',async()=>{
 const d=mock();let committed=false;
 d.put=async(n,v)=>{d.writes.push([n,v]);if(n==='tune_command')committed=true};
 d.snapshot=async()=>committed?{...ready,tune_result_id:11,tune_status:1}:{...ready};
 await d.apply(defaults);assert.deepEqual(d.writes.at(-1),['tune_command',1]);
 assert(d.writes.every(([n])=>n!=='pd_window_command'));assert.deepEqual(d.writes[0],['tune_requested_kp',floatWord(4)]);
 assert(d.writes.some(([n,v])=>n==='tune_requested_backward_kp' && v===floatWord(7.5)));
 assert(d.writes.some(([n,v])=>n==='tune_requested_backward_limit' && v===35));
 assert(d.writes.some(([n,v])=>n==='tune_requested_gyro_predict_ms' && v===120));
 assert(d.writes.some(([n,v])=>n==='tune_requested_speed_kp' && v===floatWord(1.5)));
 assert(d.writes.some(([n,v])=>n==='tune_requested_speed_ki' && v===floatWord(0.5)));
 assert(d.writes.some(([n,v])=>n==='tune_requested_speed_tilt' && v===floatWord(2)));
 assert(d.writes.some(([n,v])=>n==='tune_requested_common_boost_counts' && v===8));
 assert(d.writes.some(([n,v])=>n==='tune_requested_task_speed' && v===floatWord(3)));
 assert(d.writes.some(([n,v])=>n==='tune_requested_task_ramp_ms' && v===1000));
 const busy=mock();busy.snapshot=async()=>({...ready,pd_window_active:1});await assert.rejects(busy.apply(defaults));assert.equal(busy.writes.length,0);
});
test('uncertain preparation is sent once; protection clear never starts',async()=>{
 const d=mock();await assert.rejects(d.arm());assert.deepEqual(d.writes,[['local_alive_counter',1],['local_command',1]]);
 const c=mock();await c.clear();assert.deepEqual(c.writes,[['pd_window_command',8]]);
});
test('USB loss retains cached state; auto reconnect only connects, never applies or starts',async()=>{
 const d=mock();d.state={...ready};d.loseLink();assert.deepEqual(d.state,ready);assert.equal(d.connected,false);assert.deepEqual(d.writes,[]);
 let calls=0;d.connect=async()=>{calls++;d.connected=true};d.nextReconnect=0;await d.tick();assert.equal(calls,1);assert.deepEqual(d.writes,[]);
});
test('browser heartbeat does not stop a physical-button run, but stops a panel run',async()=>{
 const d=mock();d.lastHeartbeat=0;d.state={...ready,pd_window_active:1,local_run_source:2};d.snapshot=async()=>d.state;
 d.stopNow=async()=>{d.writes.push(['stop',9])};await d.tick();assert.deepEqual(d.writes,[]);
 d.state.local_run_source=1;await d.tick();assert.deepEqual(d.writes,[['stop',9]]);
});
test('fresh panel session feeds MCU keepalive; button session is independent',async()=>{
 const d=mock();d.state={...ready,pd_window_active:1,local_run_source:1,local_alive_counter:4};d.snapshot=async()=>d.state;
 await d.tick();assert.deepEqual(d.writes,[['local_alive_counter',5]]);
 d.writes=[];d.state.local_run_source=2;await d.tick();assert.deepEqual(d.writes,[]);
});
test('save to car requires idle state and acknowledgement; no motor command',async()=>{
 const d=mock();d.snapshot=async()=>d.writes.some(([n])=>n==='param_store_command')?{...ready,param_store_result_id:1,param_store_status:1}:{...ready,param_store_result_id:0};
 await d.persist();assert.deepEqual(d.writes,[['param_store_request_id',1],['param_store_command',1]]);
 const preparing=mock();preparing.snapshot=async()=>({...ready,local_state:1});await assert.rejects(preparing.persist());assert.deepEqual(preparing.writes,[]);
});
test('task and smooth brake each submit once; MCU owns profile; idle request is rejected',async()=>{
 for(const [command,state] of [[1,1],[9,2]]){
  const d=mock();d.snapshot=async()=>({...ready,pd_window_active:1,task_state:state,speed_kp:5,speed_ki:1,speed_tilt_limit_deg:2.5,task_result_id:d.writes.some(([n])=>n==='task_command')?3:2,task_result:1});
  await d.motion(command);assert.deepEqual(d.writes,[['task_request_id',3],['task_command',command]]);
 }
 const idle=mock();await assert.rejects(idle.motion(1));assert.deepEqual(idle.writes,[]);
 const disabled=mock();disabled.snapshot=async()=>({...ready,pd_window_active:1,task_state:1,speed_kp:5,speed_ki:1,speed_tilt_limit_deg:0});
 await assert.rejects(disabled.motion());assert.deepEqual(disabled.writes,[]);
 const uncertain=mock();uncertain.snapshot=async()=>({...ready,pd_window_active:1,task_state:1,speed_kp:5,speed_ki:1,speed_tilt_limit_deg:2.5,task_result_id:0});
 await assert.rejects(uncertain.motion());assert.deepEqual(uncertain.writes,[['task_request_id',1],['task_command',1]]);
});
test('RPC handles fragmented Tcl frames and rejects malformed reads',async()=>{
 let count=0;
 const server=net.createServer(s=>{let input='';s.on('data',b=>{input+=b.toString();if(input.includes('\x1a')){input='';count++;s.write(count===1?'0x01 ':'bad');setTimeout(()=>s.write(count===1?'2\x1a':'\x1a'),5)}})});
 await new Promise(r=>server.listen(0,'127.0.0.1',r));const rpc=await Rpc.connect(server.address().port);
 try{assert.deepEqual(await rpc.read(0x20000000,2),[1,2]);await assert.rejects(rpc.read(0x20000000,2))}finally{rpc.close();await new Promise(r=>server.close(r))}
});
test('parallel ranges, whitelisted RAM symbols and exact float readback',async()=>{
 assert.deepEqual(validateParallel({parameter:'Limit',value:100}),{symbol:'speed_parallel_limit_percent',value:100});
 assert.throws(()=>validateParallel({parameter:'Limit',value:14},13.75));
 for(const [parameter,value] of [['Kp',100.01],['Ki',-1],['Limit',100.01],['Kp','27.5'],['Kp',NaN],['__proto__',1],['pd_window_command',1]])
  assert.throws(()=>validateParallel({parameter,value}));
 const d=mock();d.manifest.controlMode='parallel';
 const input={parameter:'Ki',value:5.4};
 d.snapshot=async()=>({...ready,speed_parallel_ki:d.writes.length?Math.fround(input.value):5.5});
 await d.applyParallel(input);
 assert.deepEqual(d.writes,[['speed_parallel_ki',floatWord(input.value)]]);
 for(const changed of [{pd_window_active:1},{pd_window_command:1},{local_command:1},{local_state:1},{local_state:4},{observed_stby:1},{observed_pwm_a:1},{param_store_busy:1}]){
  const stopped=mock();stopped.manifest.controlMode='parallel';stopped.snapshot=async()=>({...ready,...changed});
  await assert.rejects(stopped.applyParallel(input),/完全停止/);assert.deepEqual(stopped.writes,[]);
 }
 const rejected=mock();rejected.manifest.controlMode='parallel';rejected.snapshot=async()=>({...ready,speed_parallel_ki:5.5});
 await assert.rejects(rejected.applyParallel(input),/读回不一致/);
 assert.equal(rejected.writes.length,1);
});
test('applying common parameters keeps rollback gains and parallel RAM separate',async()=>{
 const d=mock();d.manifest.controlMode='parallel';
 d.snapshot=async()=>({...ready,speed_kp:5,speed_ki:1,speed_tilt_limit_deg:2.5,
  ...(d.writes.some(([n])=>n==='tune_command')?{tune_result_id:11,tune_status:1}:{})});
 await d.apply(defaults);
 assert.deepEqual(d.writes.filter(([n])=>n.startsWith('tune_requested_speed')),[
  ['tune_requested_speed_kp',floatWord(5)],['tune_requested_speed_ki',floatWord(1)],['tune_requested_speed_tilt',floatWord(2.5)]
 ]);
 assert(d.writes.every(([n])=>!n.startsWith('speed_parallel_')));
});
test('parallel travel accepts timed and reverse tasks and can brake with PI off',async()=>{
 const motion=async(command,state,change={},expectedError)=>{
  const d=mock();d.manifest.controlMode='parallel';
  d.snapshot=async()=>({...ready,pd_window_active:1,task_state:state,task_requested_distance_cm:-10,
   speed_parallel_kp:27.5,speed_parallel_ki:5.5,speed_parallel_limit_percent:13.75,...change,
   task_result_id:d.writes.some(([n])=>n==='task_command')?3:2,task_result:1});
  if(expectedError){await assert.rejects(d.motion(command),expectedError);assert.deepEqual(d.writes,[])}
  else{await d.motion(command);assert.deepEqual(d.writes,[['task_request_id',3],['task_command',command]])}
 };
 await motion(1,1);
 await motion(2,4);
 await motion(9,6,{speed_parallel_limit_percent:0});
 await motion(2,1,{task_requested_distance_cm:10},/后退目标/);
 await motion(2,1,{speed_parallel_limit_percent:0},/启用速度环/);
 await motion(9,1,{},/当前状态/);
});
test('obstacle mission requires matching firmware, fresh range and its lower speed envelope',async()=>{
 const d=mock();d.manifest={controlMode:'parallel',obstacle:{triggerMm:180}};
 let change={};
 d.snapshot=async()=>({...ready,pd_window_active:1,task_state:1,avoid_state:0,task_speed_cms:3,task_ramp_ms:1000,
  task_requested_distance_cm:-10,speed_parallel_kp:25.5,speed_parallel_ki:5,speed_parallel_limit_percent:100,
  tof_valid:1,tof_range_mm:500,tof_last_ms:150,...change,
  task_result_id:d.writes.some(([n])=>n==='task_command')?1:0,task_result:1});
 await d.motion(3);assert.deepEqual(d.writes,[['task_request_id',1],['task_command',3]]);
 for(const c of [{tof_valid:0},{tof_last_ms:0,uptime_ms:301},{task_speed_cms:3.1},{task_ramp_ms:1001},{task_requested_distance_cm:10},{avoid_state:3,task_state:4}]){
  d.writes=[];change=c;await assert.rejects(d.motion(3));assert.deepEqual(d.writes,[]);
 }
 d.writes=[];change={};d.manifest.obstacle=null;await assert.rejects(d.motion(3),/不支持/);assert.deepEqual(d.writes,[]);
});
test('task mode selection is idle-only, readback-confirmed and sends no start',async()=>{
 const d=mock();d.manifest.obstacle={};let change={};
 d.snapshot=async()=>({...ready,local_state:0,local_command:0,pd_window_command:0,task_speed_cms:3,task_ramp_ms:1000,
  local_task_mode:d.writes.length?d.writes.at(-1)[1]:0,...change});
 await d.selectTaskMode(1);assert.deepEqual(d.writes,[['local_task_mode',1]]);
 for(const c of [{pd_window_active:1},{local_state:1},{task_speed_cms:5},{task_ramp_ms:1500}]){
  d.writes=[];change=c;await assert.rejects(d.selectTaskMode(1));assert.deepEqual(d.writes,[]);
 }
 d.writes=[];change={};await assert.rejects(d.selectTaskMode(2));assert.deepEqual(d.writes,[]);
});
test('cancel during obstacle dwell is accepted with speed PI disabled and only submits once',async()=>{
 const d=mock();d.manifest={controlMode:'parallel',obstacle:{}};
 d.snapshot=async()=>({...ready,pd_window_active:1,task_state:4,avoid_state:3,speed_parallel_limit_percent:0,
  task_result_id:d.writes.length?1:0,task_result:1});
 await d.motion(9);assert.deepEqual(d.writes,[['task_request_id',1],['task_command',9]]);
});
test('known firmware digests select matching layouts even with different image lengths',()=>{
 const original=Buffer.from([1,2,3,4]);
 const updated=Buffer.from([5,6,7,8,9,10,11,12]);
 const manifest=image=>({flashLength:image.length,flashSha256:createHash('sha256').update(image).digest('hex')});
 const base=manifest(original),persistent={...manifest(updated),parallelPersistent:true};
 const images=[base,persistent];
 assert.equal(matchingFirmware(Buffer.concat([original,Buffer.alloc(4,255)]),images),base);
 assert.equal(matchingFirmware(updated,images),persistent);
 updated[0]^=1;
 assert.equal(matchingFirmware(updated,images),undefined);
});
test('new parallel application marks pending save and persists only on explicit request',async()=>{
 const d=mock();d.manifest={controlMode:'parallel',parallelPersistent:true};
 d.snapshot=async()=>({...ready,speed_parallel_ki:d.writes.some(([n])=>n==='speed_parallel_ki')?2:5.5,
  ...(d.writes.some(([n])=>n==='param_store_command')?{param_store_result_id:1,param_store_status:1}:{param_store_result_id:0})});
 await d.applyParallel({parameter:'Ki',value:2});
 assert.deepEqual(d.writes,[['speed_parallel_ki',floatWord(2)],['param_store_dirty',1]]);
 assert.equal(d.view().parallelPersistent,true);
 await d.persist();
 assert.deepEqual(d.writes.slice(2),[['param_store_request_id',1],['param_store_command',1]]);
});
test('snapshot skips large trace RAM holes and still decodes fields and refreshes sample age',async()=>{
 const base=0x20000000;
 const symbols={imu_body_pitch_deg:base,uptime_ms:base+4,imu_last_sample_ms:base+8,observed_pwm_a:base+0x3800};
 const d=new Device({symbols,readNames:Object.keys(symbols)}),calls=[];
 const memory=new Map([[base,floatWord(1.25)],[base+4,200],[base+8,193],[base+0x3800,40]]);
 d.rpc={read:async(address,count)=>{
  calls.push([address,count]);
  assert(count<=3,'large unused RAM range should be skipped');
  return Array.from({length:count},(_,i)=>memory.get(address+i*4)||0);
 }};
 const s=await d.snapshot();
 assert.equal(s.imu_body_pitch_deg,1.25);assert.equal(s.observed_pwm_a,40);
 assert.equal(s.uptime_ms,200);assert.equal(s.imu_last_sample_ms,193);
 assert.deepEqual(calls,[[base,3],[base+0x3800,1],[base+8,1],[base+4,1]]);
});
test('frozen trace export decodes wrapped frames and rejects concurrent record replacement',async()=>{
 const d=mock();d.manifest.trace=true;
 d.manifest.symbols={control_trace_frames:0x20001000,control_trace_parameters:0x20000020};
 const state={...ready,control_trace_status:2,control_trace_api:1,control_trace_capacity:320,
  control_trace_frame_bytes:40,control_trace_count:2,control_trace_head:1,control_trace_total:321,control_trace_run:3};
 const data=Buffer.alloc(320*40);
 for(const [index,time] of [[319,100],[0,110]]){
  data.writeUInt32LE(time,index*40);data.writeFloatLE(-1.5,index*40+8);
  data.writeInt16LE(-20,index*40+32);data.writeUInt16LE(40,index*40+34);
  data.writeUInt16LE(0x9003,index*40+38);
 }
 d.snapshot=async()=>({...state});
 d.rpc={read:async(address,count)=>address===0x20000020?Array(12).fill(floatWord(7)):
  Array.from({length:count},(_,i)=>data.readUInt32LE(address-0x20001000+i*4))};
 const trace=await d.readTrace();
 assert.deepEqual(trace.frames.map(frame=>frame.ms),[100,110]);
 assert.equal(trace.frames[0].bodyDeg,-1.5);assert.equal(trace.frames[0].appliedCounts,-20);
 assert.equal(trace.frames[0].pwmA,10);assert.equal(trace.frames[0].flags,0x9003);
 assert.equal(trace.parameters.balance_kp,7);
 let calls=0;d.snapshot=async()=>({...state,control_trace_run:++calls===1?3:4});
 await assert.rejects(d.readTrace(),/读取时发生变化/);
 d.snapshot=async()=>({...state,pd_window_active:1});
 await assert.rejects(d.readTrace(),/先长按停机/);
});

test('wide speed limit requires matching firmware capability before RAM write',async()=>{
 const old=mock();old.manifest.controlMode='parallel';
 await assert.rejects(old.applyParallel({parameter:'Limit',value:35}),/13.75/);assert.deepEqual(old.writes,[]);
 const d=mock();d.manifest.controlMode='parallel';d.manifest.parallelLimitMax=60;
 d.snapshot=async()=>({...ready,speed_parallel_limit_percent:d.writes.length?60:13.75});
 await d.applyParallel({parameter:'Limit',value:60});
 assert.deepEqual(d.writes,[['speed_parallel_limit_percent',floatWord(60)]]);
 assert.equal(d.view().parallelLimitMax,60);
});

test('100 percent output range requires matching firmware capability',async()=>{
 assert.equal(validate({...defaults,OutputLimitPercent:100,BackwardLimitPercent:100}).OutputLimitPercent,100);
 const old=mock();await assert.rejects(old.apply({...defaults,OutputLimitPercent:100}),/60%/);assert.deepEqual(old.writes,[]);
 const d=mock();d.manifest.outputLimitMax=100;d.snapshot=async()=>({...ready,...(d.writes.some(([n])=>n==='tune_command')?{tune_result_id:11,tune_status:1}:{})});await d.apply({...defaults,OutputLimitPercent:100,BackwardLimitPercent:100});
 assert.ok(d.writes.some(([n,v])=>n==='tune_requested_limit'&&v===100));
 assert.ok(d.writes.some(([n,v])=>n==='tune_requested_backward_limit'&&v===100));
 assert.equal(d.view().outputLimitMax,100);
});

test('35 degree protection capability retains the older 15 degree guard',()=>{
 for(const body of [-34.9,34.9])assert.doesNotThrow(()=>readiness({...ready,imu_body_pitch_deg:body},35));
 for(const body of [-35.1,35.1])assert.throws(()=>readiness({...ready,imu_body_pitch_deg:body},35),/35/);
 assert.throws(()=>readiness({...ready,imu_body_pitch_deg:16}));
 const d=mock();d.manifest.tiltLimitDeg=35;assert.equal(d.view().tiltLimitDeg,35);
});


test('unsettled mission failure can be cleared without starting another travel task',async()=>{
 const d=mock();d.manifest={controlMode:'parallel',obstacle:{}};
 d.snapshot=async()=>({...ready,pd_window_active:1,task_state:8,avoid_state:7,speed_parallel_limit_percent:0,
  task_result_id:d.writes.length?1:0,task_result:1});
 await d.motion(9);assert.deepEqual(d.writes,[['task_request_id',1],['task_command',9]]);
 assert.match(d.message,/再次操作/);
 d.writes=[];await assert.rejects(d.motion(1),/当前状态/);assert.deepEqual(d.writes,[]);
});


test('mailbox firmware stages epoch before commit and uses its own acknowledgement',async()=>{
 const d=mock();d.manifest={controlMode:'parallel',taskMailboxApi:1};
 d.snapshot=async()=>({...ready,pd_window_active:1,task_state:1,speed_parallel_kp:25.5,speed_parallel_ki:5,
  speed_parallel_limit_percent:100,task_mailbox_api:1,task_panel_command:0,task_cancel_epoch:42,
  task_result_id:999,task_panel_result_id:d.writes.some(([n])=>n==='task_panel_command')?4:3,task_panel_result:1});
 await d.motion(1);assert.deepEqual(d.writes,[['task_panel_request_id',4],['task_panel_expected_epoch',42],['task_panel_command',1]]);
});

test('button cancellation rejects stale web staging without retry or fallback',async()=>{
 const d=mock();d.manifest={controlMode:'parallel',taskMailboxApi:1};
 d.snapshot=async()=>({...ready,pd_window_active:1,task_state:1,speed_parallel_kp:25.5,speed_parallel_ki:5,
  speed_parallel_limit_percent:100,task_mailbox_api:1,task_panel_command:0,task_cancel_epoch:2,
  task_panel_result_id:d.writes.some(([n])=>n==='task_panel_command')?1:0,task_panel_result:4});
 await assert.rejects(d.motion(1),/实体操作或停机取消/);assert.equal(d.writes.length,3);
 assert(d.writes.every(([n])=>n!=='task_command'));
});

test('pending mailbox blocks a second commit and mismatched API blocks writes',async()=>{
 for(const change of [{task_panel_command:1},{task_mailbox_api:2}]){
  const d=mock();d.manifest={controlMode:'parallel',taskMailboxApi:1};
  d.snapshot=async()=>({...ready,pd_window_active:1,task_state:1,speed_parallel_kp:25.5,speed_parallel_ki:5,
   speed_parallel_limit_percent:100,task_mailbox_api:1,task_panel_command:0,...change});
  await assert.rejects(d.motion(1),/接口尚未就绪/);assert.deepEqual(d.writes,[]);
 }
});


test('mailbox acknowledgement is confirmed after its ID publication',async()=>{
 const d=mock();d.manifest={controlMode:'parallel',taskMailboxApi:1};let completedReads=0;
 d.snapshot=async()=>{
  const committed=d.writes.some(([n])=>n==='task_panel_command');
  if(committed)completedReads++;
  return {...ready,pd_window_active:1,task_state:1,speed_parallel_kp:25.5,speed_parallel_ki:5,
   speed_parallel_limit_percent:100,task_mailbox_api:1,task_panel_command:0,task_cancel_epoch:2,
   task_panel_result_id:committed?1:0,task_panel_result:completedReads>=2?4:1};
 };
 await assert.rejects(d.motion(1),/实体操作或停机取消/);assert.equal(completedReads,2);assert.equal(d.writes.length,3);
});

test('review candidate identity and every RAM address match its physical build',async()=>{
 const m=JSON.parse(await readFile(new URL('./firmware-reviewfix.json',import.meta.url),'utf8'));
 const fw=new URL('../firmware/Car_TLM1_PA9_Parallel/',import.meta.url);
 const bin=await readFile(new URL('release/car_tlm1_pa9.bin',fw));
 const axf=await readFile(new URL('release/car_tlm1_pa9.axf',fw));
 const map=await readFile(new URL('evidence/car_tlm1_pa9.map',fw),'utf8');
 assert.equal(bin.length,m.flashLength);assert(bin.length<0xf800);
 assert.equal(createHash('sha256').update(bin).digest('hex'),m.flashSha256);
 assert.equal(createHash('sha256').update(axf).digest('hex'),m.axfSha256);
 const symbols=new Map([...map.matchAll(/^\s+(\w+)\s+(0x[\da-f]+)\s+Data\s+\d+/gmi)].map(x=>[x[1],Number(x[2])]));
 const offsets={fault_reject_flags:36,fault_rate_dps:48,fault_ay_g:56,fault_norm2_g2:64,fault_gyro_age_ms:72};
 for(const [name,address] of Object.entries(m.symbols)){
  assert.equal(address,name in offsets?symbols.get('pd_fault_sample')+offsets[name]:symbols.get(name),name);
  assert(address>=0x20000000&&address+4<=0x20005000&&address%4===0,name);
 }
 assert.equal(m.taskMailboxApi,1);assert.equal(m.readNames.length,177);assert.equal(new Set(m.readNames).size,177);
 for(const name of ['task_mailbox_api','task_panel_command','task_panel_request_id','task_panel_expected_epoch',
                    'task_panel_result_id','task_panel_result','task_cancel_epoch','tof_transfer_error_count','oled_transfer_error_count','oled_dirty_chunks'])
  assert(m.readNames.includes(name),name);
 assert.equal(m.obstacle.failedTaskState,8);assert.equal(m.peripherals.ioMaxUs,2000);
 const server=await readFile(new URL('./server.mjs',import.meta.url),'utf8');
 assert(server.includes("'firmware-reviewfix.json'"));
});

test('competition identity, memory and all observations match the physical image',async()=>{
 const m=JSON.parse(await readFile(new URL('./firmware-competition.json',import.meta.url),'utf8'));
 const old=JSON.parse(await readFile(new URL('./firmware-reviewfix.json',import.meta.url),'utf8'));
 const fw=new URL('../firmware/Car_Competition/',import.meta.url);
 const bin=await readFile(new URL('release/car_competition.bin',fw));
 const axf=await readFile(new URL('release/car_competition.axf',fw));
 const map=await readFile(new URL('evidence/car_competition.map',fw),'utf8');
 assert.equal(bin.length,m.flashLength);assert(bin.length<0xf800);
 assert.equal(createHash('sha256').update(bin).digest('hex'),m.flashSha256);
 assert.equal(createHash('sha256').update(axf).digest('hex'),m.axfSha256);
 assert.equal(Number(map.match(/Total ROM Size[^\r\n]*?\)\s+(\d+)/)[1]),bin.length);
 assert.equal(Number(map.match(/Total RW\s+Size[^\r\n]*?\)\s+(\d+)/)[1]),4032);
 const symbols=new Map([...map.matchAll(/^\s+(\w+)\s+(0x[\da-f]+)\s+Data\s+\d+/gmi)].map(x=>[x[1],Number(x[2])]));
 const offsets={fault_reject_flags:36,fault_rate_dps:48,fault_ay_g:56,fault_norm2_g2:64,fault_gyro_age_ms:72};
 for(const [name,address] of Object.entries(m.symbols)){
  assert.equal(address,name in offsets?symbols.get('pd_fault_sample')+offsets[name]:symbols.get(name),name);
  assert(address>=0x20000000&&address+4<=0x20005000&&address%4===0,name);
 }
 assert.equal(m.readNames.length,173);assert.equal(new Set(m.readNames).size,173);
 for(const name of m.readNames)assert(name in m.symbols);
 for(const name of ['control_trace_frames','motor_command','pulse_encoder_e1_delta'])assert(!symbols.has(name),name);
 assert(!Object.keys(m.symbols).some(n=>n.startsWith('control_trace_')));
 assert.equal(m.trace,false);assert.equal(m.manualReverse,false);assert.equal(m.buttonRepeatTimed,true);
 assert.equal(m.parallelPersistent,true);assert.equal(m.taskMailboxApi,1);
 assert.deepEqual(m.pitchUart,{tx:'PA2',peripheral:'USART2',baud:115200,dataBits:8,parity:'none',stopBits:1,
  periodMs:50,format:'PITCH=+000.000\\r\\n',invalid:'PITCH=INVALID\\r\\n',dma:'DMA1_Channel7',receiveControl:false});
 assert.equal(matchingFirmware(bin,[old,m]),m);
 const oldImage=await readFile(new URL('../firmware/Car_TLM1_PA9_Parallel/release/car_tlm1_pa9.bin',import.meta.url));
 assert.equal(matchingFirmware(oldImage,[m,old]),old);
 const corrupted=Buffer.from(bin);corrupted[corrupted.length-1]^=1;
 assert.equal(matchingFirmware(corrupted,[m,old]),undefined);
 const server=await readFile(new URL('./server.mjs',import.meta.url),'utf8');
 assert(server.includes("'firmware-competition.json'"));
});

test('competition refuses manual reverse before any device write and retains legacy capabilities',async()=>{
 const m=JSON.parse(await readFile(new URL('./firmware-competition.json',import.meta.url),'utf8'));
 const d=mock();d.manifest=m;
 d.snapshot=async()=>({...ready,pd_window_active:1,task_state:1,speed_parallel_kp:25.5,speed_parallel_ki:5,
  speed_parallel_limit_percent:100,task_requested_distance_cm:-10});
 await assert.rejects(d.motion(2),/不支持/);assert.deepEqual(d.writes,[]);
 assert.equal(d.view().manualReverse,false);assert.equal(d.view().buttonRepeatTimed,true);
 assert.equal(d.view().parallelPersistent,true);
 const legacy=mock();assert.equal(legacy.view().manualReverse,true);assert.equal(legacy.view().buttonRepeatTimed,false);
});

test('competition still sends forward and obstacle tasks through the checked mailbox',async()=>{
 const m=JSON.parse(await readFile(new URL('./firmware-competition.json',import.meta.url),'utf8'));
 for(const command of [1,3]){
  const d=mock();d.manifest=m;
  d.snapshot=async()=>({...ready,pd_window_active:1,task_state:1,speed_parallel_kp:25.5,speed_parallel_ki:5,
   speed_parallel_limit_percent:100,task_speed_cms:3,task_ramp_ms:1000,task_requested_distance_cm:-10,
   tof_valid:1,tof_range_mm:500,tof_last_ms:195,task_mailbox_api:1,task_panel_command:0,task_cancel_epoch:9,
   task_panel_result_id:d.writes.some(([n])=>n==='task_panel_command')?8:7,task_panel_result:1});
  await d.motion(command);
  assert.deepEqual(d.writes,[['task_panel_request_id',8],['task_panel_expected_epoch',9],['task_panel_command',command]]);
 }
});

test('competition blocks removed trace reads without touching the device',async()=>{
 const d=mock();d.manifest=JSON.parse(await readFile(new URL('./firmware-competition.json',import.meta.url),'utf8'));
 d.snapshot=async()=>{throw Error('Unexpected device access')};
 await assert.rejects(d.readTrace(),/不支持车载记录/);assert.deepEqual(d.writes,[]);
});

test('direction swap image and every RAM address select the new layout',async()=>{
 const m=JSON.parse(await readFile(new URL('./firmware-direction-swap.json',import.meta.url),'utf8'));
 const old=JSON.parse(await readFile(new URL('./firmware-competition.json',import.meta.url),'utf8'));
 const fw=new URL('../firmware/Car_Competition_DirectionSwap/',import.meta.url);
 const bin=await readFile(new URL('release/car_competition.bin',fw));
 const axf=await readFile(new URL('release/car_competition.axf',fw));
 const map=await readFile(new URL('evidence/car_competition.map',fw),'utf8');
 assert.equal(bin.length,m.flashLength);assert(bin.length<0xf800);
 assert.equal(createHash('sha256').update(bin).digest('hex'),m.flashSha256);
 assert.equal(createHash('sha256').update(axf).digest('hex'),m.axfSha256);
 const symbols=new Map([...map.matchAll(/^\s+(\w+)\s+(0x[\da-f]+)\s+Data\s+\d+/gmi)].map(x=>[x[1],Number(x[2])]));
 const offsets={fault_reject_flags:36,fault_rate_dps:48,fault_ay_g:56,fault_norm2_g2:64,fault_gyro_age_ms:72};
 for(const [name,address] of Object.entries(m.symbols)){
  assert.equal(address,name in offsets?symbols.get('pd_fault_sample')+offsets[name]:symbols.get(name),name);
  assert(address>=0x20000000&&address+4<=0x20005000&&address%4===0,name);
 }
 assert.equal(m.readNames.length,174);assert.equal(new Set(m.readNames).size,174);
 assert(m.readNames.includes('task_direction_reversed'));assert.equal(m.directionsReversed,true);
 assert.equal(m.parametersChanged,false);assert.equal(m.parallelPersistent,true);
 assert.equal(matchingFirmware(bin,[old,m]),m);
 const legacy=new Device(old);assert.equal(legacy.view().directionsReversed,false);
 assert((await readFile(new URL('./server.mjs',import.meta.url),'utf8')).includes("'firmware-direction-swap.json'"));
});

test('direction swap retains task commands and parameter controls',async()=>{
 const m=JSON.parse(await readFile(new URL('./firmware-direction-swap.json',import.meta.url),'utf8'));
 for(const command of [1,3]){
  const d=mock();d.manifest=m;
  d.snapshot=async()=>({...ready,pd_window_active:1,task_state:1,speed_parallel_kp:25.5,speed_parallel_ki:5,
   speed_parallel_limit_percent:100,task_speed_cms:3,task_ramp_ms:1000,task_requested_distance_cm:-10,
   tof_valid:1,tof_range_mm:500,tof_last_ms:195,task_mailbox_api:1,task_panel_command:0,task_cancel_epoch:9,
   task_panel_result_id:d.writes.some(([n])=>n==='task_panel_command')?8:7,task_panel_result:1});
  await d.motion(command);
  assert.deepEqual(d.writes,[['task_panel_request_id',8],['task_panel_expected_epoch',9],['task_panel_command',command]]);
  assert.equal(d.view().directionsReversed,true);assert.equal(d.view().parallelPersistent,true);
  assert.equal(d.view().manualReverse,false);assert.equal(d.view().buttonRepeatTimed,true);
 }
});

test('live button firmware identity, RAM layout and LED capabilities match its image',async()=>{
 const m=JSON.parse(await readFile(new URL('./firmware-live-mode.json',import.meta.url),'utf8'));
 const old=JSON.parse(await readFile(new URL('./firmware-direction-swap.json',import.meta.url),'utf8'));
 const fw=new URL('../firmware/Car_Competition_LiveMode/',import.meta.url);
 const bin=await readFile(new URL('release/car_competition.bin',fw));
 const axf=await readFile(new URL('release/car_competition.axf',fw));
 const map=await readFile(new URL('evidence/car_competition.map',fw),'utf8');
 assert.equal(bin.length,m.flashLength);assert(bin.length<0xf800);
 assert.equal(createHash('sha256').update(bin).digest('hex'),m.flashSha256);
 assert.equal(createHash('sha256').update(axf).digest('hex'),m.axfSha256);
 const symbols=new Map([...map.matchAll(/^\s+(\w+)\s+(0x[\da-f]+)\s+Data\s+\d+/gmi)].map(x=>[x[1],Number(x[2])]));
 const offsets={fault_reject_flags:36,fault_rate_dps:48,fault_ay_g:56,fault_norm2_g2:64,fault_gyro_age_ms:72};
 for(const [name,address] of Object.entries(m.symbols)){
  assert.equal(address,name in offsets?symbols.get('pd_fault_sample')+offsets[name]:symbols.get(name),name);
  assert(address>=0x20000000&&address+4<=0x20005000&&address%4===0,name);
 }
 assert.equal(m.readNames.length,177);assert.equal(new Set(m.readNames).size,177);
 for(const name of ['local_button_api','local_led_kind','observed_led_on'])assert(m.readNames.includes(name),name);
 assert.equal(matchingFirmware(bin,[old,m]),m);
 assert.equal(m.buttons.api,2);assert.equal(m.buttons.holdMs,800);assert.equal(m.buttons.powerCycleRequired,false);
 assert.equal(m.buttons.releaseRequiredAfterStop,true);assert.equal(m.obstacle.bootHoldSelect,false);
 assert.equal(m.leds.statusPin,'PC13');assert.equal(m.leds.obstaclePin,'PA7');
 assert.equal(m.leds.basicFlashes,1);assert.equal(m.leds.avoidFlashes,2);assert.equal(m.leds.faultFlashes,4);
 const d=new Device(m);assert.deepEqual(d.view().buttons,m.buttons);assert.deepEqual(d.view().leds,m.leds);
 assert.equal(new Device(old).view().buttons,null);
 assert((await readFile(new URL('./server.mjs',import.meta.url),'utf8')).includes("'firmware-live-mode.json'"));
});

test('live-mode firmware keeps panel mode changes idle-only without reconnection or motor command',async()=>{
 const m=JSON.parse(await readFile(new URL('./firmware-live-mode.json',import.meta.url),'utf8'));
 const d=mock();d.manifest=m;let mode=0,active=0;
 d.snapshot=async()=>({...ready,pd_window_active:active,local_state:0,local_command:0,pd_window_command:0,
  task_speed_cms:3,task_ramp_ms:1000,local_task_mode:mode});
 d.put=async(n,v)=>{d.writes.push([n,v]);if(n==='local_task_mode')mode=v};
 await d.selectTaskMode(1);await d.selectTaskMode(0);
 assert.deepEqual(d.writes,[['local_task_mode',1],['local_task_mode',0]]);assert.equal(d.connected,true);
 active=1;await assert.rejects(d.selectTaskMode(1),/先关闭电机/);assert.equal(d.writes.length,2);
 assert.equal(m.parallelPersistent,true);assert.equal(m.directionsReversed,true);assert.equal(m.parametersChanged,false);
});

test('offline balance diagnostic identity and all 188 addresses match its physical image',async()=>{
 const m=JSON.parse(await readFile(new URL('./firmware-balance-diag.json',import.meta.url),'utf8'));
 const old=JSON.parse(await readFile(new URL('./firmware-live-mode.json',import.meta.url),'utf8'));
 const fw=new URL('../firmware/Car_Competition_BalanceDiag/',import.meta.url);
 const bin=await readFile(new URL('release/car_competition.bin',fw));
 const axf=await readFile(new URL('release/car_competition.axf',fw));
 const map=await readFile(new URL('evidence/car_competition.map',fw),'utf8');
 assert.equal(bin.length,m.flashLength);assert(bin.length<0xf800);
 assert.equal(createHash('sha256').update(bin).digest('hex'),m.flashSha256);
 assert.equal(createHash('sha256').update(axf).digest('hex'),m.axfSha256);
 assert.equal(Number(map.match(/Total ROM Size[^\r\n]*?\)\s+(\d+)/)[1]),31412);
 assert.equal(Number(map.match(/Total RW\s+Size[^\r\n]*?\)\s+(\d+)/)[1]),6280);
 const symbols=new Map([...map.matchAll(/^\s+(\w+)\s+(0x[\da-f]+)\s+Data\s+(\d+)/gmi)].map(x=>[x[1],{address:Number(x[2]),bytes:Number(x[3])}]));
 const offsets={fault_reject_flags:36,fault_rate_dps:48,fault_ay_g:56,fault_norm2_g2:64,fault_gyro_age_ms:72};
 for(const [name,address] of Object.entries(m.symbols)){
  assert.equal(address,name in offsets?symbols.get('pd_fault_sample').address+offsets[name]:symbols.get(name).address,name);
  assert(address>=0x20000000&&address+4<=0x20005000&&address%4===0,name);
 }
 for(const [name,bytes] of [['balance_diag_frames',2048],['balance_diag_last',64],['balance_diag_parameters',64]]){
  assert.equal(symbols.get(name).bytes,bytes);assert(m.symbols[name]+bytes<=0x20005000);
 }
 assert.equal(m.readNames.length,188);assert.equal(new Set(m.readNames).size,188);
 for(const name of m.readNames)assert(name in m.symbols);
 assert.equal(matchingFirmware(bin,[old,m]),m);
 assert((await readFile(new URL('./server.mjs',import.meta.url),'utf8')).includes("'firmware-balance-diag.json'"));
});

test('offline balance diagnostics retain live buttons, directions and saved tuning capabilities',async()=>{
 const m=JSON.parse(await readFile(new URL('./firmware-balance-diag.json',import.meta.url),'utf8'));
 const old=JSON.parse(await readFile(new URL('./firmware-live-mode.json',import.meta.url),'utf8'));
 assert.deepEqual(m.buttons,old.buttons);assert.deepEqual(m.leds,old.leds);
 assert.equal(m.parametersChanged,false);assert.equal(m.directionsReversed,true);assert.equal(m.parallelPersistent,true);
 assert.deepEqual(m.balanceDiag,{api:1,frameBytes:64,capacity:32,periodMs:500,parameterFloats:16,
  freezeOnStop:true,recordingStartsOnlyWithPhysicalOrExplicitBalanceStart:true,storedInRam:true,clearedByReset:true});
 assert.equal(m.manualReverse,false);assert.equal(m.trace,false);
 const d=new Device(m);assert.equal(d.view().parallelPersistent,true);assert.deepEqual(d.view().buttons,m.buttons);
});

test('stop hold image identity and every observed address match its physical map',async()=>{
 const m=JSON.parse(await readFile(new URL('./firmware-stop-hold.json',import.meta.url),'utf8'));
 const old=JSON.parse(await readFile(new URL('./firmware-balance-diag.json',import.meta.url),'utf8'));
 const fw=new URL('../firmware/Car_Competition_StopHold/',import.meta.url);
 const bin=await readFile(new URL('release/car_competition.bin',fw));
 const axf=await readFile(new URL('release/car_competition.axf',fw));
 const map=await readFile(new URL('evidence/car_competition.map',fw),'utf8');
 assert.equal(bin.length,m.flashLength);assert(bin.length<0xf800);
 assert.equal(createHash('sha256').update(bin).digest('hex'),m.flashSha256);
 assert.equal(createHash('sha256').update(axf).digest('hex'),m.axfSha256);
 const symbols=new Map([...map.matchAll(/^\s+(\w+)\s+(0x[\da-f]+)\s+Data\s+\d+/gmi)].map(x=>[x[1],Number(x[2])]));
 const offsets={fault_reject_flags:36,fault_rate_dps:48,fault_ay_g:56,fault_norm2_g2:64,fault_gyro_age_ms:72};
 for(const [name,address] of Object.entries(m.symbols)){
  assert.equal(address,name in offsets?symbols.get('pd_fault_sample')+offsets[name]:symbols.get(name),name);
  assert(address>=0x20000000&&address+4<=0x20005000&&address%4===0,name);
 }
 assert.equal(m.readNames.length,179);assert.equal(new Set(m.readNames).size,179);
 assert(m.readNames.includes('position_hold_api'));assert(m.readNames.includes('task_stop_ramp_ms'));
 assert(!symbols.has('balance_diag_frames'));assert(!symbols.has('param_store_diag_write'));
 assert.equal(matchingFirmware(bin,[old,m]),m);
 assert((await readFile(new URL('./server.mjs',import.meta.url),'utf8')).includes("'firmware-stop-hold.json'"));
});

test('stop hold advertises earlier capture without changing saved tuning, direction or buttons',async()=>{
 const m=JSON.parse(await readFile(new URL('./firmware-stop-hold.json',import.meta.url),'utf8'));
 const old=JSON.parse(await readFile(new URL('./firmware-live-mode.json',import.meta.url),'utf8'));
 assert.deepEqual(m.buttons,old.buttons);assert.deepEqual(m.leds,old.leds);
 assert.equal(m.parametersChanged,false);assert.equal(m.directionsReversed,true);assert.equal(m.parallelPersistent,true);
 assert.equal(m.positionHold.api,2);assert.equal(m.positionHold.freeCm,0.5);assert.equal(m.positionHold.stableMs,100);
 assert.equal(m.positionHold.enterAngleDeg,4);assert.equal(m.positionHold.enterRateDps,25);assert.equal(m.positionHold.enterSpeedCms,6);
 assert.equal(m.positionHold.exitAngleDeg,old.positionHold.exitAngleDeg);
 assert.equal(m.positionHold.exitRateDps,old.positionHold.exitRateDps);assert.equal(m.positionHold.exitSpeedCms,old.positionHold.exitSpeedCms);
 assert(m.positionHold.parkingImmediate&&m.positionHold.activeWhileBraking&&m.positionHold.activeWhileSettling&&m.positionHold.stopTargetRetained);
 assert.deepEqual(m.stopControl,{brakeRampMs:300,distanceArrivalZeroReference:true,stoppedConfirmationMs:500});
 assert.equal(m.trace,false);assert.equal(m.balanceDiag,undefined);
 const d=new Device(m);assert.equal(d.view().parallelPersistent,true);assert.deepEqual(d.view().positionHold,m.positionHold);
});
