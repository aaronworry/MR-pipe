import yaml

class GetMap():
    def __init__(self, file_path):
        self.layer_num = 0
        self.layer_height = []
        self.map_dict = self.map_read(file_path)
    
    def map_read(self, file_path):
        with open(file_path, 'r') as file:
            data = file.read()
            result = yaml.load(data, Loader=yaml.FullLoader)
            return result['map']
        
        
        
if __name__ == "__main__":
    Map = GetMap("case1.yaml")
    print(Map.map_dict)