export const fields = {
  Kp: [0, 20, 0], Kd: [0, 2, 0], OutputLimitPercent: [1, 100, 1],
  BackwardKp: [0, 20, 0], BackwardLimitPercent: [1, 100, 1],
  RunMs: [100, 30000, 1], RampStepMs: [0, 200, 1],
  RightBoostPercent: [0, 10, 0.25], BoostMaxBasePercent: [1, 60, 1],
  CommonBoostPercent: [0, 6, 0.25],
  TaskSpeedCmS:[0.5,10,0], TaskRampMs:[300,2000,1],
  GyroPredictMs: [30, 300, 1], SpeedKp:[0,20,0], SpeedKi:[0,10,0], SpeedTiltLimitDeg:[0,5,0]
};
export const defaults = {Kp:4,BackwardKp:7.5,Kd:0.05,OutputLimitPercent:25,BackwardLimitPercent:35,RunMs:6000,RampStepMs:8,RightBoostPercent:2,BoostMaxBasePercent:5,CommonBoostPercent:2,GyroPredictMs:120,SpeedKp:1.5,SpeedKi:0.5,SpeedTiltLimitDeg:2,TaskSpeedCmS:3,TaskRampMs:1000};
export const parallelFields=Object.freeze({Kp:['speed_parallel_kp',100],Ki:['speed_parallel_ki',50],Limit:['speed_parallel_limit_percent',100]});
export function validateParallel(input,limitMax=100) {
  const name=input?.parameter;
  if(typeof name!=='string'||!Object.hasOwn(parallelFields,name))throw Error('未知的并行参数');
  const [symbol,fieldMax]=parallelFields[name],max=name==='Limit'?Math.min(fieldMax,limitMax):fieldMax,value=input.value;
  if(typeof value!=='number'||!Number.isFinite(value)||value<0||value>max)throw Error(`并行 ${name} 必须在 0～${max} 内`);
  return {symbol,value:Math.fround(value)};
}
export function validate(input) {
  if (!input || typeof input !== 'object' || Array.isArray(input)) throw Error('参数格式错误');
  if (Object.keys(input).some(k => !Object.hasOwn(fields,k))) throw Error('存在未知参数');
  const out = {};
  for (const [k,[min,max,step]] of Object.entries(fields)) {
    const v=input[k];
    if(typeof v !== 'number' || !Number.isFinite(v) || v<min || v>max || (step && !Number.isInteger(v/step))) throw Error(`${k} 必须在 ${min}～${max} 内${step?'，步长 '+step:''}`);
    out[k]=v;
  }
  if((Math.max(out.OutputLimitPercent,out.BackwardLimitPercent)-1)*out.RampStepMs>out.RunMs) throw Error('前进或后退渐升时间超过运行时长');
  return out;
}
export function parseIni(text) {
  const out={};
  for(const raw of text.replace(/^\uFEFF/,'').split(/\r?\n/)) {
    const line=raw.split(/[;#]/)[0].trim(); if(!line)continue;
    const m=/^(\w+)\s*=\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:e[+-]?\d+)?)$/i.exec(line);
    if(!m)throw Error(`无法解析：${line}`);
    if(Object.hasOwn(out,m[1]))throw Error(`参数重复：${m[1]}`);
    out[m[1]]=Number(m[2]);
  }
  if(!Object.hasOwn(out,'CommonBoostPercent'))out.CommonBoostPercent=defaults.CommonBoostPercent;
  for(const k of ['TaskSpeedCmS','TaskRampMs'])if(!Object.hasOwn(out,k))out[k]=defaults[k];
  return validate(out);
}
export function formatIni(input) {
 const c=validate(input);
 return '; 平衡车通用参数，UTF-8。修改后在面板按“载入文件”，再按“应用到车”。\n'+
 '; 保存文件存电脑，应用改RAM；保存到车后掉电保留。均不启动。\n'+
 '; Kp：前倾比例增益，输出百分点/度，0～20。\n'+`Kp=${c.Kp}\n`+
 '; BackwardKp：后倾比例增益，输出百分点/度，0～20；小角度后倾更早加力。\n'+`BackwardKp=${c.BackwardKp}\n`+
 '; Kd：每度/秒角速度对应的输出百分点，0～2；倾倒时追赶、回正时制动。\n'+`Kd=${c.Kd}\n`+
 '; 前进时两轮各自的实际输出上限（占空比%），整数1～100。\n'+`OutputLimitPercent=${c.OutputLimitPercent}\n`+
 '; 后退时两轮各自的实际输出上限（占空比%），整数1～100。\n'+`BackwardLimitPercent=${c.BackwardLimitPercent}\n`+
 '; 旧调试时长兼容字段100～30000；P34持续平衡，不按该值关闭电机。\n'+`RunMs=${c.RunMs}\n`+
 '; 启动阶段每释放1个百分点的毫秒数，整数0～200；0关闭渐升。\n'+
 '; 从基础1%起，各方向(上限-1)*本值后释放该方向上限；减小输出、归零、停机不等待。\n'+`RampStepMs=${c.RampStepMs}\n`+
 '; 右轮低输出额外补偿，百分点0～10，步长0.25；前后对称，基础0到2%渐入，基础0时不补偿。\n'+`RightBoostPercent=${c.RightBoostPercent}\n`+
 '; 保持完整补偿的基础输出阈值（%），整数1～60；之后平台衔接。\n'+`BoostMaxBasePercent=${c.BoostMaxBasePercent}\n`+
 '; 双轮公共摩擦补偿上限，0～6百分点，步长0.25；0回退P32，实际按请求平滑渐入。\n'+`CommonBoostPercent=${c.CommonBoostPercent}\n`+
 '; 旧预测时限兼容字段，30～300；P29起已停用超时停机，此值不影响控制。\n'+`GyroPredictMs=${c.GyroPredictMs}\n`+
 '; 以下三个速度环字段仅用于串级固件；并行速度 PI 由独立 RAM 参数调节。\n'+
 '; 速度比例增益：度/(1000计数/秒)，0～20，默认1.5。\n'+`SpeedKp=${c.SpeedKp}\n`+
 '; 速度积分增益：度/1000计数，0～10，默认0.5。\n'+`SpeedKi=${c.SpeedKi}\n`+
 '; 速度环最大目标倾角，0～5度，默认2；0关闭速度环且不能启动行驶任务。\n'+`SpeedTiltLimitDeg=${c.SpeedTiltLimitDeg}\n`+
 '; 五秒任务目标速度0.5～10cm/s，初值3；不是实测保证速度。\n'+`TaskSpeedCmS=${c.TaskSpeedCmS}\n`+
 '; 任务加速和减速各自的时长300～2000ms，初值1000；总任务仍为5秒。\n'+`TaskRampMs=${c.TaskRampMs}\n`;
}
