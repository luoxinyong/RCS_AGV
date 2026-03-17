## 🚀 部署流程

---

### 1. 前端项目打包

```
npm run build
```

### 2. 将打包后的 dist 文件夹改名为 html，上传到服务器目录：

```
/home/hf/local/docker-config/nginx/html/
```

<br>

## Docker 配置 Nginx 容器

---

### 1. 创建映射目录

```
mkdir -p /home/hf/local/docker-config/nginx/conf
mkdir -p /home/hf/local/docker-config/nginx/log
```

### 2. 复制文件

```
# 预生成: 容器，复制完成后需要删除
docker run --name nginx -p 80:80 -d nginx


# 将nginx.conf复制到宿主机
docker cp nginx:/etc/nginx/nginx.conf /home/hf/local/docker-config/nginx/conf/nginx.conf

# 将conf.d复制到宿主机
docker cp nginx:/etc/nginx/conf.d /home/hf/local/docker-config/nginx/conf/conf.d

# 将html复制到宿主机
docker cp nginx:/usr/share/nginx/html /home/hf/local/docker-config/nginx/


# 找到nginx对应的容器id
docker ps -a
# 关闭该容器
docker stop nginx
# 删除该容器
docker rm nginx
```

### 3. 修改 nginx.conf 文件

```
user  nginx;
worker_processes  auto;

error_log  /var/log/nginx/error.log notice;
pid        /run/nginx.pid;


events {
    worker_connections  1024;
}


http {
    include       /etc/nginx/mime.types;
    default_type  application/octet-stream;

    log_format  main  '$remote_addr - $remote_user [$time_local] "$request" '
                      '$status $body_bytes_sent "$http_referer" '
                      '"$http_user_agent" "$http_x_forwarded_for"';

    access_log  /var/log/nginx/access.log  main;

    sendfile        on;
    #tcp_nopush     on;

    keepalive_timeout  65;

    #gzip  on;

    # include /etc/nginx/conf.d/*.conf;
    server {
        listen       80;
        server_name  localhost;

        #access_log  /var/log/nginx/host.access.log  main;

        location / {
            root   /usr/share/nginx/html;
            index  index.html index.htm;
            try_files $uri $uri/ /index.html; # 重要，否则路由跳转有问题
        }

        #error_page  404              /404.html;

        # redirect server error pages to the static page /50x.html
        #
        error_page   500 502 503 504  /50x.html;
        location = /50x.html {
            root   /usr/share/nginx/html;
        }
    }
}
```

### 4. 运行 Nginx 容器

```
docker run --name nginx -p 80:80 -d \
-v /home/hf/local/docker-config/nginx/conf/nginx.conf:/etc/nginx/nginx.conf \
-v /home/hf/local/docker-config/nginx/conf/conf.d:/etc/nginx/conf.d \
-v /home/hf/local/docker-config/nginx/log:/var/log/nginx \
-v /home/hf/local/docker-config/nginx/html:/usr/share/nginx/html \
nginx
```
