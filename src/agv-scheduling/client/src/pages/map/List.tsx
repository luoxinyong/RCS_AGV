import { useEffect } from "react";
import { List, Card } from "antd";
import { useNavigate } from "react-router-dom";
import { Building2, Cuboid } from "lucide-react";
import { localStorageKey } from "@/config";
import { mapApi } from "@/network/api";
import useApi from "@/hooks/useApi";
import type { Building } from "@/types/building";

const MapList = () => {
  const navigate = useNavigate();
  const { execute, data: buildings } = useApi<Building[]>();

  useEffect(() => {
    execute(mapApi.getMaps);
  }, [execute]);

  const onClickBuilding = (building: number, floor: number) => {
    localStorage.setItem(localStorageKey.building, building.toString());
    localStorage.setItem(localStorageKey.floor, floor.toString());
    navigate(`/map/editor`);
  };

  return (
    <div className="px-4 h-full overflow-y-auto">
      {buildings?.map((building) => (
        <div key={building.no}>
          <div className="flex items-center my-4">
            <Building2 color="black" size={20} />
            <h3 className="text-lg font-bold ml-3">{building.no}号楼</h3>
          </div>
          <List
            grid={{ gutter: 20, xs: 1, sm: 1, md: 2, lg: 2, xl: 2, xxl: 3 }}
            dataSource={building.floors}
            renderItem={(floor) => (
              <List.Item key={floor.no}>
                <Card
                  cover={
                    <div
                      className="bg-blue-500 p-6 overflow-hidden cursor-pointer"
                      onClick={() => onClickBuilding(building.no, floor.no)}
                    >
                      <img
                        className="h-44 mx-auto transition-transform duration-300 hover:scale-110"
                        src={`/assets/svg/${building.no}-${floor.no}.svg`}
                        alt={`${building.no}#${floor.no}`}
                      />
                    </div>
                  }
                >
                  <Card.Meta
                    title={
                      <div className="flex items-center">
                        <Cuboid color="#333" size={16} />
                        <span className="text-[#333] ml-2">{`${building.no}#${floor.no}`}</span>
                      </div>
                    }
                    description={floor.description}
                  />
                </Card>
              </List.Item>
            )}
          />
        </div>
      ))}
    </div>
  );
};

export default MapList;
