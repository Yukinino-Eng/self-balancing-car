import net from 'node:net';
export class Rpc {
  static async connect(port) {
    const socket=net.createConnection({host:'127.0.0.1',port});
    await new Promise((resolve,reject)=>{socket.once('connect',resolve);socket.once('error',reject)});
    return new Rpc(socket);
  }
  constructor(socket) {
    this.socket=socket;this.buffer='';this.pending=null;
    socket.setNoDelay(true);
    socket.on('data',data=>{this.buffer+=data.toString();let i;
      while((i=this.buffer.indexOf('\x1a'))>=0){const value=this.buffer.slice(0,i);this.buffer=this.buffer.slice(i+1);const p=this.pending;if(p){this.pending=null;clearTimeout(p.timer);p.resolve(value)}}
    });
    socket.on('error',e=>this.fail(e));socket.on('close',()=>this.fail(Error('ST-Link连接已断开')));
  }
  fail(e){if(this.pending){clearTimeout(this.pending.timer);this.pending.reject(e);this.pending=null}}
  command(cmd) {
    if(this.pending)throw Error('RPC并发请求');
    return new Promise((resolve,reject)=>{
      const timer=setTimeout(()=>{this.fail(Error('ST-Link响应超时；未自动重发命令'));this.socket.destroy()},3000);
      this.pending={resolve,reject,timer};this.socket.write(cmd+'\x1a');
    });
  }
  async read(address,count){
    const out=await this.command(`read_memory 0x${address.toString(16)} 32 ${count}`);
    const values=out.trim().split(/\s+/);
    if(values.length!==count || values.some(v=>! /^(?:0x[\da-f]+|\d+)$/i.test(v)))throw Error('读取失败：'+out.slice(0,160));
    return values.map(Number);
  }
  async write(address,value){
    const out=await this.command(`write_memory 0x${address.toString(16)} 32 {${value>>>0}}`);
    if(out.trim())throw Error('写入未确认：'+out.slice(0,160));
  }
  close(){this.socket.destroy()}
}
