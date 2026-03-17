# ========================
# 1. 构建阶段
# ========================
FROM node:22 AS build
WORKDIR /src

# 先复制依赖清单，避免每次都重新 npm install
COPY package*.json ./
RUN npm install --omit=dev

# 拷贝源代码并构建
COPY . .
RUN npm run build

# ========================
# 2. 运行阶段
# ========================
FROM node:22-slim
WORKDIR /app

# 只拷贝运行需要的文件
COPY --from=build /src/node_modules ./node_modules
COPY --from=build /src/dist ./dist
COPY package*.json ./
COPY .env .env

EXPOSE 3000
CMD ["node", "dist/index.js"]
