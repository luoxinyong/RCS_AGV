import { servers, localStorageKey } from "@/config";
import axios, {
  AxiosError,
  AxiosResponse,
  InternalAxiosRequestConfig,
} from "axios";

const http = axios.create({
  baseURL: servers.apiServer,
  timeout: 10000,
  headers: {
    "Content-Type": "application/json",
  },
});

// Request interceptor
http.interceptors.request.use(
  (config: InternalAxiosRequestConfig) => {
    const token = localStorage.getItem(localStorageKey.token);
    if (token) {
      config.headers.Authorization = token;
    }
    return config;
  },
  (error: AxiosError) => {
    return Promise.reject(error);
  }
);

// Response interceptor
http.interceptors.response.use(
  (response: AxiosResponse) => {
    if (response?.data?.status == 401) {
      // Handle unauthorized access
      localStorage.removeItem(localStorageKey.token);
      if (window.location.pathname !== "/login") {
        window.location.href = "/login";
      }
    } else if (response?.data?.status == 403) {
      window.location.href = "/auth";
    }
    return response.data;
  },
  (error: AxiosError) => {
    return Promise.reject(error);
  }
);

export default http;
