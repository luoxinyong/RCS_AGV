import { isTauri } from "@tauri-apps/api/core";
import { writeText } from "@tauri-apps/plugin-clipboard-manager";

/** 对象根据它的键排序 */
export function groupBy<T>(array: T[], key: keyof T): { [key: string]: T[] } {
  return array.reduce((result, item) => {
    const groupKey = String(item[key]);
    if (!result[groupKey]) {
      result[groupKey] = [];
    }
    result[groupKey].push(item);
    return result;
  }, {} as { [key: string]: T[] });
}

/** 获取指定范围内的随机数 */
export function randomInRange(min: number, max: number): number {
  return Math.floor(Math.random() * (max - min + 1)) + min;
}

/** 导出JSON文件 */
export function exportJsonFile(data: unknown) {
  const jsonData = JSON.stringify(data);
  const blob = new Blob([jsonData], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = "data.json";
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
}

/** 复制到剪切板 */
export async function copyTextToClipboard(text: string) {
  // 判断是否在 Tauri 环境
  if (isTauri()) {
    try {
      await writeText(text);
      return true;
    } catch (err) {
      console.error("Tauri clipboard failed:", err);
      return false;
    }
  }

  // fallback: 浏览器端（仅在 HTTPS / localhost 生效）
  if ("clipboard" in navigator && window.isSecureContext) {
    try {
      await navigator.clipboard.writeText(text);
      return true;
    } catch (err) {
      console.error("navigator.clipboard failed:", err);
    }
  }

  // fallback: execCommand（老旧浏览器）
  try {
    const ta = document.createElement("textarea");
    ta.style.cssText = "position:fixed;top:-1000px;opacity:0;";
    ta.value = text;
    document.body.appendChild(ta);
    ta.select();
    const success = document.execCommand("copy");
    document.body.removeChild(ta);
    return success;
  } catch (err) {
    console.error("execCommand fallback failed:", err);
    return false;
  }
}

// /** 复制到剪切板 */
// export async function copyTextToClipboard(text: string) {
//   if ("clipboard" in navigator && window.isSecureContext) {
//     await navigator.clipboard.writeText(text);
//   } else {
//     const ta = document.createElement("textarea");
//     ta.style.cssText = "position:fixed; top:-1000px; opacity: 0;";
//     ta.value = text;
//     document.body.appendChild(ta);
//     ta.select();
//     document.execCommand("copy", true, text);
//     document.body.removeChild(ta);
//   }
// }
