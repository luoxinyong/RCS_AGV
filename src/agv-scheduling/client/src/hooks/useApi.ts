import { useCallback, useState } from "react";
import { AxiosError, AxiosResponse } from "axios";

interface UseApiState<T> {
  data: T | null;
  error: Error | null;
  loading: boolean;
}

export default function useApi<T>() {
  const [state, setState] = useState<UseApiState<T>>({
    data: null,
    error: null,
    loading: false,
  });

  const execute = useCallback(
    async (apiCall: () => Promise<AxiosResponse<T>>) => {
      setState((prev) => ({ ...prev, loading: true, error: null }));

      try {
        const result = await apiCall();
        setState({ data: result.data, loading: false, error: null });
      } catch (error) {
        setState((prev) => ({
          ...prev,
          loading: false,
          error: new Error(
            error instanceof AxiosError
              ? error.response?.data?.message || error.message
              : "An unexpected error occurred"
          ),
        }));
        throw error;
      }
    },
    []
  );

  return { ...state, execute };
}
