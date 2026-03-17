import { Button, Result } from "antd";
import { useNavigate } from "react-router-dom";

export default function Analysis() {
  const navigate = useNavigate();

  const onClickBackHome = () => {
    navigate("/", { replace: true });
  };

  return (
    <div className="h-full flex items-center justify-center">
      <Result
        status="403"
        subTitle="没有访问权限！"
        extra={
          <Button type="primary" size="large" onClick={onClickBackHome}>
            返回首页
          </Button>
        }
      />
    </div>
  );
}
