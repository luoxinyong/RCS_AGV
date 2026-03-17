import { Pool } from "pg";
import APP_CONFIG from "../config/app";

export const pg = new Pool({
  host: APP_CONFIG.PG_HOST,
  port: parseInt(APP_CONFIG.PG_PORT),
  user: APP_CONFIG.PG_USER,
  password: APP_CONFIG.PG_PASSWORD,
  database: APP_CONFIG.PG_DATABASE,
});

export async function initPostgres() {
  try {
    // 检查并创建users表
    await createUsersTable();

    // 检查并创建logs表
    await createLogsTable();

    // 检查并创建devices表
    await createDevicesTable();

    // 检查并创建tasks表
    await createTasksTable();

    // 检查并创建alarms表
    await createAlarmsTable();

    // 检查并创建buildings表
    await createBuildingsTable();

    // 检查并创建Floors表
    await createFloorsTable();

    // 检查并创建doors表
    await createDoorsTable();

    console.log("✅ 数据库表结构检查完成");
  } catch (error) {
    console.error("❌ 初始化数据库表结构失败:", error);
    throw error;
  }
}

// 创建users表
async function createUsersTable() {
  try {
    // 检查表是否存在
    const checkTableQuery = `
      SELECT EXISTS (
        SELECT FROM information_schema.tables 
        WHERE table_schema = 'public' AND table_name = 'users'
      );
    `;
    const result = await pg.query(checkTableQuery);
    const tableExists = result.rows[0]?.exists || false;
    if (!tableExists) {
      // 创建表
      const createTableQuery = `
        CREATE TABLE users (
          id SERIAL PRIMARY KEY,
          employee_no VARCHAR(50) NOT NULL UNIQUE,
          employee_name VARCHAR(100) NOT NULL,
          password VARCHAR(255) NOT NULL,
          permission VARCHAR(50) NOT NULL DEFAULT 'user',
          created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
          updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
        );
      `;
      await pg.query(createTableQuery);

      // 创建默认管理员用户
      await pg.query(
        `INSERT INTO users (employee_no, employee_name, password, permission) 
         VALUES ($1, $2, $3, $4)`,
        ["admin", "管理员", "300283", "admin"]
      );
    }
  } catch (error) {
    console.error("❌ 创建users表失败:", error);
    throw error;
  }
}

// 创建logs表
async function createLogsTable() {
  try {
    // 检查表是否存在
    const checkTableQuery = `
      SELECT EXISTS (
        SELECT FROM information_schema.tables 
        WHERE table_schema = 'public' AND table_name = 'logs'
      );
    `;
    const result = await pg.query(checkTableQuery);
    const tableExists = result.rows[0]?.exists || false;
    if (!tableExists) {
      // 创建表
      const createTableQuery = `
        CREATE TABLE logs (
          id BIGSERIAL PRIMARY KEY,
          method VARCHAR(10) NOT NULL,
          path VARCHAR(255) NOT NULL,
          status SMALLINT NOT NULL,
          duration INTEGER NOT NULL,
          auth JSONB,
          query JSONB,
          body JSONB,
          response JSONB,
          created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
        );
      `;
      await pg.query(createTableQuery);
    }
  } catch (error) {
    console.error("❌ 创建logs表失败:", error);
    throw error;
  }
}

// 创建devices表
async function createDevicesTable() {
  try {
    // 检查表是否存在
    const checkTableQuery = `
      SELECT EXISTS (
        SELECT FROM information_schema.tables 
        WHERE table_schema = 'public' AND table_name = 'devices'
      );
    `;
    const result = await pg.query(checkTableQuery);
    const tableExists = result.rows[0]?.exists || false;
    if (!tableExists) {
      // 创建表
      const createTableQuery = `
        CREATE TABLE devices (
          id SERIAL PRIMARY KEY,
          name VARCHAR(100) NOT NULL,
          ip VARCHAR(45) NOT NULL,
          port INTEGER NOT NULL,
          enabled BOOLEAN NOT NULL DEFAULT true,
          remark VARCHAR(255),
          created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
          updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
        );
      `;
      await pg.query(createTableQuery);
    }
  } catch (error) {
    console.error("❌ 创建devices表失败:", error);
    throw error;
  }
}

// 创建tasks表
async function createTasksTable() {
  try {
    // 检查表是否存在
    const checkTableQuery = `
      SELECT EXISTS (
        SELECT FROM information_schema.tables 
        WHERE table_schema = 'public' AND table_name = 'tasks'
      );
    `;
    const result = await pg.query(checkTableQuery);
    const tableExists = result.rows[0]?.exists || false;
    if (!tableExists) {
      // 创建表
      const createTableQuery = `
        CREATE TABLE tasks (
          id BIGSERIAL PRIMARY KEY,
          agv_id INTEGER NOT NULL,
          type VARCHAR(20) NOT NULL,
          duration INTEGER NOT NULL,
          code TEXT,
          body JSONB,
          created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
          updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
        );
      `;
      await pg.query(createTableQuery);
    }
  } catch (error) {
    console.error("❌ 创建tasks表失败:", error);
    throw error;
  }
}

// 创建alarms表
async function createAlarmsTable() {
  try {
    // 检查表是否存在
    const checkTableQuery = `
      SELECT EXISTS (
        SELECT FROM information_schema.tables 
        WHERE table_schema = 'public' AND table_name = 'alarms'
      );
    `;
    const result = await pg.query(checkTableQuery);
    const tableExists = result.rows[0]?.exists || false;
    if (!tableExists) {
      // 创建表
      const createTableQuery = `
        CREATE TABLE alarms (
          id BIGSERIAL PRIMARY KEY,
          agv_id INTEGER NOT NULL,
          tr_check_state INTEGER,
          tr_check_error INTEGER,
          err1 INTEGER,
          err2 INTEGER,
          err3 INTEGER,
          err4 INTEGER,
          err5 INTEGER,
          warnings INTEGER,
          code TEXT,
          body JSONB,
          created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
          updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
        );
      `;
      await pg.query(createTableQuery);
    }
  } catch (error) {
    console.error("❌ 创建alarms表失败:", error);
    throw error;
  }
}

// 创建buildings表
async function createBuildingsTable() {
  try {
    // 检查表是否存在
    const checkTableQuery = `
      SELECT EXISTS (
        SELECT FROM information_schema.tables 
        WHERE table_schema = 'public' AND table_name = 'buildings'
      );
    `;
    const result = await pg.query(checkTableQuery);
    const tableExists = result.rows[0]?.exists || false;
    if (!tableExists) {
      // 创建表
      const createTableQuery = `
        CREATE TABLE buildings (
          id SERIAL PRIMARY KEY,
          no INTEGER NOT NULL UNIQUE,
          width INTEGER NOT NULL,
          height INTEGER,
          origin JSONB,
          created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
          updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
        );
      `;
      await pg.query(createTableQuery);
    }
  } catch (error) {
    console.error("❌ 创建buildings表失败:", error);
    throw error;
  }
}

// 创建floors表
async function createFloorsTable() {
  try {
    // 检查表是否存在
    const checkTableQuery = `
      SELECT EXISTS (
        SELECT FROM information_schema.tables 
        WHERE table_schema = 'public' AND table_name = 'floors'
      );
    `;
    const result = await pg.query(checkTableQuery);
    const tableExists = result.rows[0]?.exists || false;
    if (!tableExists) {
      // 创建表
      const createTableQuery = `
        CREATE TABLE floors (
          id SERIAL PRIMARY KEY,
          no INTEGER NOT NULL UNIQUE,
          description VARCHAR(255),
          agv_origin JSONB,
          points JSONB,
          lines JSONB,
          paths JSONB,
          building_no INTEGER NOT NULL REFERENCES buildings(no),
          created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
          updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
        );
      `;
      await pg.query(createTableQuery);
    }
  } catch (error) {
    console.error("❌ 创建floors表失败:", error);
    throw error;
  }
}

// 创建doors表
async function createDoorsTable() {
  try {
    // 检查表是否存在
    const checkTableQuery = `
      SELECT EXISTS (
        SELECT FROM information_schema.tables 
        WHERE table_schema = 'public' AND table_name = 'doors'
      );
    `;
    const result = await pg.query(checkTableQuery);
    const tableExists = result.rows[0]?.exists || false;
    if (!tableExists) {
      // 创建表
      const createTableQuery = `
        CREATE TABLE doors (
          id SERIAL PRIMARY KEY,
          ip VARCHAR(45) NOT NULL,
          description VARCHAR(255),
          created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
          updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
        );
      `;
      await pg.query(createTableQuery);
    }
  } catch (error) {
    console.error("❌ 创建doors表失败:", error);
    throw error;
  }
}
