import {spawn} from 'node:child_process';
import net from 'node:net';
import {createHash} from 'node:crypto';
import {Rpc} from './rpc.mjs';
import {validate,validateParallel} from './config.mjs';
const delay=ms=>new Promise(r=>setTimeout(r,ms));
const floatNames=new Set(['balance_kp','balance_backward_kp','balance_kd','imu_body_pitch_deg','imu_upright_reference_deg','imu_pitch_rate_dps','pd_window_requested_percent','hold_right_boost_percent','fault_rate_dps','fault_norm2_g2','fault_ay_g','speed_kp','speed_ki','speed_tilt_limit_deg','speed_left_cps','speed_right_cps','speed_filtered_cps','speed_integral_deg','speed_target_body_deg','speed_wheel_difference_cps','speed_wheel_correction_percent','hold_common_boost_percent','task_speed_cms','task_reference_cps','task_distance_cm','speed_error_cps']);
for(const n of ['position_hold_offset_cm','position_hold_target_cms','speed_effective_reference_cps','speed_wheel_integral_percent','balance_p_percent','balance_d_percent','speed_parallel_kp','speed_parallel_ki','speed_parallel_limit_percent','speed_parallel_p_percent','speed_parallel_i_percent','speed_parallel_output_percent','task_requested_distance_cm','task_target_distance_cm','task_distance_error_cm'])floatNames.add(n);
floatNames.add('avoid_reverse_cm');
export function floatWord(v){const b=Buffer.alloc(4);b.writeFloatLE(v);return b.readUInt32LE()}
export function matchingFirmware(image,manifests){
 return manifests.find(m=>m.flashLength<=image.length &&
   createHash('sha256').update(image.subarray(0,m.flashLength)).digest('hex')===m.flashSha256);
}
export function readiness(s,tiltLimitDeg=15){
 if(s.pd_window_active)throw Error('车正在运行');
 if(s.pd_window_latched)throw Error('保护已锁存，请先检查状态并单独解除保护');
 if(s.motor_guard_state!==1 || (s.imu_fusion_mode!==1 && s.imu_fusion_mode!==2) || !s.imu_valid || !s.imu_fusion_valid)throw Error('等待有效的融合姿态');
 if(!Number.isFinite(s.imu_body_pitch_deg) || !Number.isFinite(s.imu_pitch_rate_dps) || ((s.uptime_ms-s.imu_last_sample_ms)>>>0)>30 || Math.abs(s.imu_body_pitch_deg)>tiltLimitDeg)throw Error(`请扶稳车身：倾角须在±${tiltLimitDeg}°且姿态数据有效`);
}
export class Device {
 constructor(manifest,alternates=[]){this.manifest=manifest;this.manifests=[manifest,...alternates];this.connected=false;this.connecting=false;this.jobs=[];this.busy=false;this.generation=0;this.countdownUntil=0;this.lastHeartbeat=Date.now();this.state=null;this.message='等待 ST-Link；页面和参数会保留';this.logs='';this.autoReconnect=true;this.nextReconnect=0;}
 heartbeat(){this.lastHeartbeat=Date.now()}
 queue(task,urgent=false){return new Promise((resolve,reject)=>{this.jobs[urgent?'unshift':'push']({task,resolve,reject});this.drain()})}
 async drain(){if(this.busy)return;this.busy=true;while(this.jobs.length){const job=this.jobs.shift();try{job.resolve(await job.task())}catch(e){this.message=e.message;job.reject(e)}}this.busy=false}
 address(name){const a=this.manifest.symbols[name];if(!Number.isInteger(a))throw Error('缺少固件符号 '+name);return a}
 async put(name,value){if(!this.connected)throw Error('未连接或固件尚未核验');await this.rpc.write(this.address(name),value)}
 async snapshot(){
   const names=this.manifest.readNames;
   const start=Math.min(...names.map(n=>this.address(n))),end=Math.max(...names.map(n=>this.address(n)))+4;
   if(start<0x20000000 || end>0x20005000)throw Error('RAM符号范围错误');
   // Skip large RAM buffers between observed variables.
   const addresses=[...new Set(names.map(n=>this.address(n)))].sort((a,b)=>a-b),spans=[];
   for(const address of addresses){
     const previous=spans.at(-1);
     if(previous && address-previous.end<=256)previous.end=address+4;
     else spans.push({start:address,end:address+4});
   }
   const values=new Map(),s={};
   for(const span of spans){
     const words=await this.rpc.read(span.start,(span.end-span.start)/4);
     words.forEach((word,i)=>values.set(span.start+i*4,word));
   }
   for(const n of names){const w=values.get(this.address(n));if(floatNames.has(n)){const b=Buffer.alloc(4);b.writeUInt32LE(w);s[n]=b.readFloatLE()}else s[n]=['pd_window_applied_percent','pd_window_applied_counts','hold_sync_applied_counts','hold_sync_saturation_error_counts'].includes(n)?w|0:w}
   // Refresh the age pair together: bulk SWD reads span several MCU ticks.
   s.imu_last_sample_ms=(await this.rpc.read(this.address('imu_last_sample_ms'),1))[0];
   s.uptime_ms=(await this.rpc.read(this.address('uptime_ms'),1))[0];
   s.sampledAt=Date.now();this.state=s;return s;
 }
 async connect(){return this.queue(async()=>{
   if(this.connected)return this.state;
   this.connecting=true;this.message='正在连接 ST-Link 并核验固件';
   try{
    const server=net.createServer();await new Promise((r,j)=>server.once('error',j).listen(0,'127.0.0.1',r));const port=server.address().port;await new Promise(r=>server.close(r));
    const m=this.manifest;
    const scripts=process.env.OPENOCD_SCRIPTS;
    if(!scripts)throw Error('请设置 OPENOCD_SCRIPTS 为 OpenOCD 的 scripts 目录');
    this.child=spawn(process.env.OPENOCD_EXE || m.openocd,['-s',scripts,'-c','bindto 127.0.0.1','-c',`tcl_port ${port}`,'-c','gdb_port disabled','-c','telnet_port disabled','-f','interface/stlink.cfg','-c','transport select swd','-f','target/stm32f1x.cfg','-c','adapter speed 50','-c','reset_config none','-c','init'],{windowsHide:true,stdio:['ignore','pipe','pipe']});
    this.child.on('error',e=>{this.logs+=e.message});
    for(const stream of [this.child.stdout,this.child.stderr])stream.on('data',d=>{this.logs=(this.logs+d).slice(-12000)});
    const child=this.child;
    child.on('exit',()=>{if(this.child===child)this.loseLink()});
    for(let i=0;i<50;i++){try{this.rpc=await Rpc.connect(port);break}catch{await delay(100)}if(child.exitCode!==null || this.child!==child)throw Error('ST-Link连接失败，请检查下载器连接；'+this.logs.slice(-800))}
    if(!this.rpc || this.child!==child)throw Error('ST-Link连接超时或已断开');
    const image=Buffer.alloc(Math.max(...this.manifests.map(candidate=>candidate.flashLength)));
    for(let offset=0;offset<image.length;offset+=1024){const values=await this.rpc.read(0x08000000+offset,Math.min(256,(image.length-offset)/4));values.forEach((v,i)=>image.writeUInt32LE(v,offset+i*4))}
    const matched=matchingFirmware(image,this.manifests);
    if(!matched)throw Error('固件版本不匹配，已禁止写入；需要与本面板匹配的并行 PID 固件');
    if((await this.rpc.read(matched.symbols.tune_api_version,1))[0]!==matched.api || matched.api!==7)throw Error('固件接口版本不匹配，已禁止写入');
    this.manifest=matched;
    this.connected=true;const s=await this.snapshot();
    this.message=s.pd_window_active?'已连接，正在读取；先停止再调参':'已连接，固件核验通过；未启动或覆盖参数';
    return this.state;
   }catch(e){this.loseLink();throw e}
   finally{this.connecting=false}
 })}
 async readTrace(){
   const generation=this.generation;
   return this.queue(async()=>{
     if(!this.connected || !this.manifest.trace)throw Error('当前固件不支持车载记录');
     const before=await this.snapshot();
     if(before.pd_window_active || [1,4].includes(before.local_state))throw Error('请先长按停机，再读取记录');
     if(before.control_trace_status!==2 || !before.control_trace_count)throw Error('暂无停机记录，请先完成一次实体按钮测试');
     const capacity=before.control_trace_capacity,size=before.control_trace_frame_bytes;
     if(before.control_trace_api!==1 || capacity!==320 || size!==40 ||
        before.control_trace_count>capacity || before.control_trace_head>=capacity)throw Error('车载记录布局不匹配');
     const start=this.address('control_trace_frames'),bytes=capacity*size;
     if(start<0x20000000 || start+bytes>0x20005000)throw Error('车载记录地址超出RAM');
     const buffer=Buffer.alloc(bytes);
     for(let offset=0;offset<bytes;offset+=1024){
       if(generation!==this.generation)throw Error('读取已取消');
       const words=await this.rpc.read(start+offset,Math.min(256,(bytes-offset)/4));
       words.forEach((word,i)=>buffer.writeUInt32LE(word,offset+i*4));
     }
     const parameterAddress=this.address('control_trace_parameters');
     if(parameterAddress<0x20000000 || parameterAddress+48>0x20005000)throw Error('参数快照地址超出RAM');
     const savedParameters=await this.rpc.read(parameterAddress,12);
     const parameterBuffer=Buffer.alloc(48);
     savedParameters.forEach((word,i)=>parameterBuffer.writeUInt32LE(word,i*4));
     const after=await this.snapshot();
     if(generation!==this.generation || after.pd_window_active ||
        ['control_trace_status','control_trace_head','control_trace_count','control_trace_total','control_trace_run']
        .some(name=>after[name]!==before[name]))throw Error('记录在读取时发生变化，请停机后重读');
     const frames=[];
     for(let i=0;i<before.control_trace_count;i++){
       const index=(before.control_trace_head-before.control_trace_count+capacity+i)%capacity,offset=index*size;
       frames.push({ms:buffer.readUInt32LE(offset),sample:buffer.readUInt32LE(offset+4),
         bodyDeg:buffer.readFloatLE(offset+8),rateDps:buffer.readFloatLE(offset+12),
         requestPercent:buffer.readFloatLE(offset+16),leftCps:buffer.readFloatLE(offset+20),
         rightCps:buffer.readFloatLE(offset+24),speedOutput:buffer.readFloatLE(offset+28),
         appliedCounts:buffer.readInt16LE(offset+32),pwmA:buffer.readUInt16LE(offset+34)/4,
         pwmB:buffer.readUInt16LE(offset+36)/4,flags:buffer.readUInt16LE(offset+38)});
     }
     const names=['balance_kp','balance_backward_kp','balance_kd','speed_parallel_kp','speed_parallel_ki',
       'speed_parallel_limit_percent','hold_output_limit_percent','hold_backward_limit_percent',
       'hold_common_boost_percent','hold_right_boost_percent','hold_start_step_ms','imu_upright_reference_deg'];
     return {firmware:this.manifest.variant,run:before.control_trace_run,stopStatus:before.control_trace_stop_status,
       totalSamples:before.control_trace_total,retainedSamples:frames.length,samplePeriodMs:10,
       parameters:Object.fromEntries(names.map((name,i)=>[name,parameterBuffer.readFloatLE(i*4)])),frames};
   });
 }
 async apply(input){const c=validate(input),generation=this.generation;return this.queue(async()=>{
   if(generation!==this.generation)throw Error('操作已被停止按钮取消');
   const s=await this.snapshot();if(s.pd_window_active || s.local_state===1 || s.local_state===4)throw Error('请先停止，再应用参数');
   const outputMax=this.manifest.outputLimitMax??60;
   if(c.OutputLimitPercent>outputMax || c.BackwardLimitPercent>outputMax)throw Error(`当前固件前后 PWM 上限最高 ${outputMax}%`);
   if(this.manifest.controlMode==='parallel'){
     // These flash fields are reserved for the cascade rollback image.
     c.SpeedKp=s.speed_kp;c.SpeedKi=s.speed_ki;c.SpeedTiltLimitDeg=s.speed_tilt_limit_deg;
     validate(c);
   }
   const request=(s.tune_result_id+1)>>>0;
   const values={tune_requested_kp:floatWord(c.Kp),tune_requested_backward_kp:floatWord(c.BackwardKp),tune_requested_kd:floatWord(c.Kd),tune_requested_limit:c.OutputLimitPercent,tune_requested_backward_limit:c.BackwardLimitPercent,tune_requested_window_ms:c.RunMs,tune_requested_step_ms:c.RampStepMs,tune_requested_boost_counts:c.RightBoostPercent*4,tune_requested_boost_base:c.BoostMaxBasePercent,tune_requested_common_boost_counts:c.CommonBoostPercent*4,tune_requested_gyro_predict_ms:c.GyroPredictMs,tune_requested_speed_kp:floatWord(c.SpeedKp),tune_requested_speed_ki:floatWord(c.SpeedKi),tune_requested_speed_tilt:floatWord(c.SpeedTiltLimitDeg),tune_requested_task_speed:floatWord(c.TaskSpeedCmS),tune_requested_task_ramp_ms:c.TaskRampMs,tune_request_id:request};
   for(const [n,v] of Object.entries(values)){if(generation!==this.generation)throw Error('参数提交已取消');await this.put(n,v)}
   if(generation!==this.generation)throw Error('参数提交已取消');
   await this.put('tune_command',1);
   for(let i=0;i<20;i++){await delay(20);const a=await this.snapshot();if(a.tune_result_id===request){if(a.tune_status!==1)throw Error(`固件拒绝：状态${a.tune_status}，参数序号${a.tune_error_field}`);this.message='参数已整组生效，保持停止';return a}}
   throw Error('参数提交未收到确认，请读取当前值；不会自动重发');
 })}
 async applyParallel(input){
   if(this.manifest.controlMode!=='parallel')throw Error('该固件不支持并行参数');
   const {symbol,value}=validateParallel(input,this.manifest.parallelLimitMax??13.75),generation=this.generation;
   return this.queue(async()=>{
     if(!this.connected||generation!==this.generation)throw Error('未连接或调参已取消');
     const s=await this.snapshot();
     if(s.pd_window_active || s.pd_window_command || s.local_command || [1,4].includes(s.local_state) ||
        s.observed_stby || s.observed_pwm_a || s.observed_pwm_b || s.param_store_busy)throw Error('请在完全停止后调节并行参数');
     if(generation!==this.generation)throw Error('调参已取消');
     await this.put(symbol,floatWord(value));
     const a=await this.snapshot();
     if(a[symbol]!==value)throw Error('参数读回不一致，请查看车上实际值');
     if(this.manifest.parallelPersistent){
       if(s[symbol]!==value){
         await this.put('param_store_dirty',1);
         a.param_store_dirty=1;
       }
       this.message='并行参数已生效，点击“保存到车”可掉电保留';
     }else this.message='并行参数已生效 · 仅本次通电';
     return a;
   });
 }
 async arm(){const generation=this.generation;return this.queue(async()=>{
   if(generation!==this.generation)throw Error('启动已取消');const s=await this.snapshot();readiness(s,this.manifest.tiltLimitDeg??15);
   if(s.local_state===1 || s.local_state===4)throw Error('已在准备启动');
   if(generation!==this.generation)throw Error('启动已取消');
   await this.put('local_alive_counter',((s.local_alive_counter||0)+1)>>>0);
   if(generation!==this.generation)throw Error('启动已取消');
   await this.put('local_command',1); // One preparation request; the MCU owns the timer.
   await delay(50);const a=await this.snapshot();
   if(a.local_state!==1)throw Error('启动准备未确认；不会重发，请读取状态');
   this.heartbeat();this.message='3秒准备，请扶到±2°并松开按钮；进入平衡后再短按启动任务，长按0.8秒关闭电机';return a;
 })}
 async selectTaskMode(mode){const generation=this.generation;return this.queue(async()=>{
   if(!this.manifest.obstacle)throw Error('当前固件未安装避障任务');
   if(mode!==0&&mode!==1)throw Error('任务模式只能是基本或避障');
   if(generation!==this.generation)throw Error('操作已取消');
   const s=await this.snapshot();
   if(s.pd_window_active||[1,4].includes(s.local_state)||s.local_command||s.pd_window_command)throw Error('请先关闭电机，再选择任务模式');
   if(mode===1&&!(s.task_speed_cms>=0.5&&s.task_speed_cms<=3&&s.task_ramp_ms>=300&&s.task_ramp_ms<=1000))throw Error('避障速度须0.5～3cm/s、渐变须300～1000ms，请先应用对应任务参数');
   await this.put('local_task_mode',mode);const a=await this.snapshot();
   if(a.local_task_mode!==mode)throw Error('模式读回未确认');
   this.message=mode?'已选避障模式；尚未启动，下次进入平衡后再短按任务键':'已选基本五秒任务；尚未启动';return a;
 })}
 async motion(command=1){const generation=this.generation;return this.queue(async()=>{
   if(generation!==this.generation)throw Error('任务已取消');
   const s=await this.snapshot();
   const parallel=this.manifest.controlMode==='parallel';
   const avoiding=!!this.manifest.obstacle&&[1,2,3,4,6].includes(s.avoid_state);
   if(![1,2,3,9].includes(command)||(command===2&&(!parallel||this.manifest.manualReverse===false))||(command===3&&!this.manifest.obstacle))throw Error('当前固件不支持该任务');
   if(!s.pd_window_active || (command!==9&&avoiding) || !(command===9?(avoiding?[s.task_state]:(parallel?[2,6,8]:[2])):[1,4]).includes(s.task_state))throw Error('当前状态不能执行该任务操作');
   const enabled=parallel?(s.speed_parallel_limit_percent>0&&(s.speed_parallel_kp>0||s.speed_parallel_ki>0)):
     (s.speed_tilt_limit_deg>0&&(s.speed_kp>0||s.speed_ki>0));
   if(command!==9&&!enabled)throw Error('请先启用速度环');
   if(command===2&&!(s.task_requested_distance_cm>=-20&&s.task_requested_distance_cm<=-5))throw Error('车上后退目标应在 −20～−5 cm');
   if(command===3){
     if(!(s.task_speed_cms>=0.5&&s.task_speed_cms<=3&&s.task_ramp_ms>=300&&s.task_ramp_ms<=1000))throw Error('避障速度须0.5～3cm/s、渐变须300～1000ms');
     if(!(s.task_requested_distance_cm>=-20&&s.task_requested_distance_cm<=-5))throw Error('避障后退目标应在 −20～−5 cm');
     if(!s.tof_valid||!s.tof_range_mm||((s.uptime_ms-s.tof_last_ms)>>>0)>300)throw Error('请先对准墙面，等待有效测距');
   }
   const mailbox=this.manifest.taskMailboxApi===1;
   if(mailbox&&(s.task_mailbox_api!==1||s.task_panel_command))throw Error('任务接口尚未就绪，请重新读取状态');
   const request=((mailbox?s.task_panel_result_id:s.task_result_id)+1)>>>0;
   const resultId=mailbox?'task_panel_result_id':'task_result_id',result=mailbox?'task_panel_result':'task_result';
   await this.put(mailbox?'task_panel_request_id':'task_request_id',request);
   if(mailbox)await this.put('task_panel_expected_epoch',s.task_cancel_epoch);
   if(generation!==this.generation)throw Error('任务已取消');
   await this.put(mailbox?'task_panel_command':'task_command',command); // Commit exactly once.
   for(let i=0;i<20;i++){await delay(20);let a=await this.snapshot();if(a[resultId]===request){
     // After seeing the published ID, read again: a bulk snapshot can span
     // publication and pair the new ID with an older status word.
     if(mailbox){a=await this.snapshot();if(a[resultId]!==request)throw Error('任务确认发生变化，请重新读取状态');}
     if(a[result]!==1){
       if(mailbox&&a[result]===4)throw Error('任务已被实体操作或停机取消，请重新读取状态');
       if(command===3&&a.avoid_reason===1)throw Error('测距未就绪，避障未启动，仍保持平衡');
       if(command===3&&a.avoid_reason===7)throw Error('避障任务参数超出允许范围，仍保持平衡');
       throw Error('固件拒绝任务，状态'+a[result]);
     }
     this.message=command===1?'五秒前进任务已开始，结束后继续平衡':command===2?'定距后退已开始，停车后继续平衡':command===3?'避障任务已开始：18cm触发停车，停稳平衡5秒，再定距后退':s.task_state===8?'任务异常已清除，保持平衡；再次操作才开始任务':avoiding?'正在结束避障任务，平衡控制继续运行':'正在平滑停车，平衡控制继续运行';return a;
   }}
   throw Error('任务未收到确认；不会自动重发，请读取当前状态');
 })}
 async persist(){const generation=this.generation;return this.queue(async()=>{
   if(generation!==this.generation)throw Error('保存已取消');
   const s=await this.snapshot();
   if(s.pd_window_active || s.pd_window_command || s.local_command || s.local_state===1 || s.local_state===4 ||
      s.observed_stby || s.observed_pwm_a || s.observed_pwm_b || s.param_store_busy)throw Error('请先停止，再保存到车');
   const request=(s.param_store_result_id+1)>>>0;
   await this.put('param_store_request_id',request);
   if(generation!==this.generation)throw Error('保存已取消');
   await this.put('param_store_command',1);
   for(let i=0;i<30;i++){await delay(30);const a=await this.snapshot();if(a.param_store_result_id===request){if(a.param_store_status!==1)throw Error('保存到车失败，状态'+a.param_store_status);this.message='车上当前参数已保存，断电后可恢复';return a}}
   throw Error('保存未收到确认；不会自动重写，请检查状态');
 })}
 stop(){this.generation++;this.countdownUntil=0;return this.queue(()=>this.stopNow(),true)}
 async stopNow(){
   await this.put('pd_window_command',9);
   for(let i=0;i<15;i++){await delay(20);const s=await this.snapshot();if(!s.pd_window_command && !s.pd_window_active && !s.observed_stby && !s.observed_pwm_a && !s.observed_pwm_b){
     const stby=(await this.rpc.read(0x4001080c,1))[0]&256, pwm=await this.rpc.read(0x4000043c,2);
     if(!stby && pwm.every(v=>v===0)){this.message='已停止，硬件输出为0';return s}
   }}throw Error('停止未获硬件确认，请关闭绿色板电源');
 }
 clear(){const generation=this.generation;return this.queue(async()=>{
   if(generation!==this.generation)throw Error('操作已取消');
   const s=await this.snapshot();if(s.pd_window_active || this.countdownUntil)throw Error('先停止，再解除保护');
   await this.put('pd_window_command',8);await delay(50);const a=await this.snapshot();
   if(a.pd_window_latched || a.pd_window_fault || a.pd_window_status===3)throw Error('保护尚未解除：扶稳、等待至少1秒后再试');
   this.message='保护已解除，保持停止';return a;
 })}
 async tick(){
   if(this.busy || this.connecting)return;
   if(!this.connected){if(this.autoReconnect && Date.now()>=this.nextReconnect){this.nextReconnect=Date.now()+3000;try{await this.connect()}catch(e){if(e.message.startsWith('固件')){this.message=e.message;this.autoReconnect=false}else this.message='等待 ST-Link；页面和参数已保留，插入后自动连接'}}return}
   if(Date.now()-this.lastHeartbeat>2500 && this.state?.local_run_source===1 && (this.state?.local_state===1 || this.state?.local_state===4 || this.state?.pd_window_active)){await this.stop();this.message='面板连接中断，面板发起的运行已停止';return}
   try{await this.queue(async()=>{
     if(this.state?.local_run_source===1 && (this.state?.pd_window_active || this.state?.local_state===1 || this.state?.local_state===4) && Date.now()-this.lastHeartbeat<=2500)
       await this.put('local_alive_counter',((this.state.local_alive_counter||0)+1)>>>0);
     return this.snapshot();
   })}catch(e){this.loseLink();throw e}
 }
 loseLink(){this.connected=false;this.countdownUntil=0;this.generation++;this.nextReconnect=Date.now()+3000;const child=this.child;this.child=null;this.rpc?.close();this.rpc=null;child?.kill();this.message='ST-Link已断开，页面与参数保留'}
 async disconnect(){this.autoReconnect=false;this.generation++;this.countdownUntil=0;return this.queue(async()=>{
   try{if(this.connected)await this.stopNow()}finally{this.loseLink();this.message='已停止并断开；点击连接可恢复自动连接'}
 },true)}
 view(){return {buttons:this.manifest.buttons??null,leds:this.manifest.leds??null,directionsReversed:this.manifest.directionsReversed===true,manualReverse:this.manifest.manualReverse!==false,buttonRepeatTimed:this.manifest.buttonRepeatTimed===true,obstacle:this.manifest.obstacle??null,peripherals:this.manifest.peripherals??null,positionHold:this.manifest.positionHold??null,wheelSync:this.manifest.wheelSync??null,tiltLimitDeg:this.manifest.tiltLimitDeg??15,outputLimitMax:this.manifest.outputLimitMax??60,parallelLimitMax:this.manifest.parallelLimitMax??13.75,parallelPersistent:this.manifest.parallelPersistent===true,connected:this.connected,connecting:this.connecting,countdownUntil:this.connected&&this.state?.local_state===1?Date.now()+this.state.local_remaining_ms:0,message:this.message,state:this.state}}
}
