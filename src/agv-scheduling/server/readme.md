## 🚀 部署流程

---

1. 在当前目录，执行脚本：`.\build-and-export.ps1`。

2. 成功后会生成一个文件: **_agv-server.tar_**。
   把这个 **_.tar_** 文件以及 **_.env_** 文件，拷贝到离线服务器上的挂载目录 ( /home/hf/local/docker-config/agv-server )

3. 在离线服务器上，加载镜像：

   ```
   cd /home/hf/local/docker-config/agv-server
   docker load -i agv-server.tar
   ```

4. 在该目录，继续执行:
   ```
   # cd /home/hf/local/docker-config/agv-server
   docker run -d -p 3000:3000 --name agv-server \
     -v /home/hf/local/docker-config/agv-server/.env:/app/.env \
     agv-server
   ```
