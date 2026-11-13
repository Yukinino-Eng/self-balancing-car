import http from 'node:http';
import fs from 'node:fs/promises';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {randomBytes} from 'node:crypto';
import {spawn} from 'node:child_process';
import {Device} from './device.mjs';
import {defaults,parseIni,formatIni,validate} from './config.mjs';
const root=path.dirname(fileURLToPath(import.meta.url));
function openBrowser(url){const child=spawn('explorer.exe',[url],{stdio:'ignore'});child.on('error',e=>console.error('浏览器未打开，请复制地址：'+url+'；'+e.message));child.unref()}
if(process.argv.includes('--open')){
 try{
  const session=JSON.parse(await fs.readFile(path.join(root,'session.json'),'utf8'));
  const u=new URL(session.url);
  const existing=await fetch(u.origin+'/api/state',{headers:{'x-panel-token':u.hash.slice(1)},signal:AbortSignal.timeout(2000)});
  if(existing.ok){console.log('已打开现有面板：'+session.url);openBrowser(session.url);process.exitCode=0;await new Promise(r=>setTimeout(r,500));process.exit(0)}
 }catch{}
}
const manifest=JSON.parse(await fs.readFile(path.join(root,'firmware.json'),'utf8'));
const alternates=[];
for(const name of ['firmware-persistent.json','firmware-trace.json','firmware-wide.json','firmware-full.json','firmware-tilt35.json','firmware-zero049.json','firmware-directspeed.json','firmware-syncpi.json','firmware-positionhold.json','firmware-peripherals.json','firmware-peripherals-fast.json','firmware-peripherals-diag.json','firmware-peripherals-gpio18.json','firmware-peripherals-reset.json','firmware-peripherals-ad19.json','firmware-peripherals-retry.json','firmware-peripherals-stable.json','firmware-obstacle.json','firmware-peripherals-recovery.json','firmware-peripherals-irq.json','firmware-reviewfix.json','firmware-competition.json','firmware-direction-swap.json','firmware-live-mode.json','firmware-balance-diag.json','firmware-stop-hold.json']){
 try{alternates.push(JSON.parse(await fs.readFile(path.join(root,name),'utf8')))}
 catch(e){if(e.code!=='ENOENT')throw e}
}
const device=new Device(manifest,alternates),token=randomBytes(24).toString('hex');
const configPath=path.join(root,'参数.ini');
try{await fs.access(configPath)}catch{await fs.writeFile(configPath,formatIni(defaults),'utf8')}
const assets={'/':['index.html','text/html; charset=utf-8'],'/app.js':['app.js','text/javascript; charset=utf-8'],'/style.css':['style.css','text/css; charset=utf-8']};
let origin;
const server=http.createServer(async(req,res)=>{
 res.setHeader('Cache-Control','no-store');res.setHeader('X-Content-Type-Options','nosniff');
 res.setHeader('Content-Security-Policy',"default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'");
 const reply=(status,body)=>{res.writeHead(status,{'Content-Type':'application/json; charset=utf-8'});res.end(JSON.stringify(body))};
 try{
  if(req.headers.host!==new URL(origin).host)throw Error('Host验证失败');
  const url=new URL(req.url,origin);
  if(req.method==='GET' && assets[url.pathname]){const [file,mime]=assets[url.pathname];res.writeHead(200,{'Content-Type':mime});res.end(await fs.readFile(path.join(root,file)));return}
  if(req.headers['x-panel-token']!==token || (req.headers.origin && req.headers.origin!==origin)){reply(403,{error:'请从本次启动的面板链接进入'});return}
  if(req.method==='GET' && url.pathname==='/api/state'){device.heartbeat();reply(200,device.view());return}
  if(req.method==='GET' && url.pathname==='/api/trace'){reply(200,await device.readTrace());return}
  if(req.method==='GET' && url.pathname==='/api/config'){reply(200,{config:parseIni(await fs.readFile(configPath,'utf8'))});return}
  if(req.method!=='POST'){reply(404,{error:'未找到'});return}
  if(req.headers['content-type']!=='application/json')throw Error('请求格式错误');
  let raw='';for await(const part of req){raw+=part;if(raw.length>4096)throw Error('请求过大')}
  const body=JSON.parse(raw||'{}');device.heartbeat();
  switch(url.pathname){
   case '/api/connect':device.autoReconnect=true;await device.connect();break;
   case '/api/disconnect':await device.disconnect();break;
   case '/api/apply':await device.apply(body);break;
   case '/api/parallel':await device.applyParallel(body);break;
   case '/api/start':await device.arm();break;
   case '/api/task':await device.motion(1);break;
   case '/api/avoid':await device.motion(3);break;
   case '/api/task-mode':await device.selectTaskMode(body.mode);break;
   case '/api/reverse':await device.motion(2);break;
   case '/api/brake':await device.motion(9);break;
   case '/api/stop':await device.stop();break;
   case '/api/clear':await device.clear();break;
   case '/api/save':{const c=validate(body),tmp=configPath+'.tmp';await fs.writeFile(tmp,formatIni(c),'utf8');await fs.rename(tmp,configPath);break}
   case '/api/persist':await device.persist();break;
   case '/api/shutdown':await device.disconnect();reply(200,{ok:true});setTimeout(()=>{server.close();process.exit(0)},100);return;
   default:reply(404,{error:'未找到'});return;
  }
  reply(200,{ok:true,...device.view()});
 }catch(e){reply(400,{error:e.message,...device.view()})}
});
await new Promise((resolve,reject)=>server.once('error',reject).listen(Number(process.env.BALANCE_PANEL_PORT||0),'127.0.0.1',resolve));
origin=`http://127.0.0.1:${server.address().port}`;
await fs.writeFile(path.join(root,'session.json'),JSON.stringify({pid:process.pid,url:origin+'/#'+token,startedAt:new Date().toISOString()},null,2));
console.log('面板：'+origin+'/#'+token);
if(process.argv.includes('--open'))openBrowser(origin+'/#'+token);
setInterval(()=>device.tick().catch(e=>{device.message=e.message;if(!device.rpc || device.rpc.socket.destroyed){device.connected=false;device.countdownUntil=0;device.generation++}}),200).unref();
for(const signal of ['SIGINT','SIGTERM'])process.on(signal,async()=>{try{await device.disconnect()}finally{server.close();process.exit(0)}});
