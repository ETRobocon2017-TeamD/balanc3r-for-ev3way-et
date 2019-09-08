import json

def load_settings():
    json_file = open('./settings.json', 'r')
    obj = json.load(json_file)
    return obj

def __settings_main():
    print(load_settings())

if __name__ == '__main__':
    __settings_main()
