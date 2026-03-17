import ModbusRTU from "modbus-serial";
import type { ReadRegisterResult } from "modbus-serial/ModbusRTU";

export default class ModbusClient {
  private client: ModbusRTU | undefined;
  private isConnected = false;

  constructor(
    public readonly id: string,
    public readonly ip: string,
    public readonly port: number = 502
  ) {}

  public async connect() {
    if (this.isConnected === false) {
      this.client = new ModbusRTU();
      await this.client.connectTCP(this.ip, {
        port: this.port,
        timeout: 9000,
      });
      this.client.setTimeout(3000);
      this.client.setID(1); // 设备编号根据实际情况设置
      this.isConnected = true;
    }
  }

  public destroy() {
    if (this.isConnected) {
      try {
        this.client?.destroy();
      } catch (error) {}
      this.client = undefined;
      this.isConnected = false;
    }
  }

  public async readRegisters(
    type: "holding" | "input",
    startAddress: number,
    length: number
  ) {
    await this.connect();

    let response: ReadRegisterResult;
    switch (type) {
      case "holding":
        response = await this.client!.readHoldingRegisters(
          startAddress,
          length
        );
        break;

      case "input":
        response = await this.client!.readInputRegisters(startAddress, length);
        break;

      default:
        throw new Error(`Unsupported register type: ${type}`);
    }

    return response.buffer;
  }

  public async writeSingleRegister(address: number, value: number) {
    await this.connect();
    await this.client?.writeRegister(address, value);
  }

  public async writeSingleRegisters(address: number, value: Buffer) {
    await this.connect();
    await this.client?.writeRegisters(address, value);
  }

  public async readCoils(startAddress: number, length: number) {
    await this.connect();
    const response = await this.client!.readCoils(startAddress, length);
    return response.data;
  }

  public async writeSingleCoil(address: number, value: boolean) {
    await this.connect();
    await this.client?.writeCoil(address, value);
  }
}
