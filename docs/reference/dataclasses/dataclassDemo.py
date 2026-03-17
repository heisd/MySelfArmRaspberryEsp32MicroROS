from dataclasses import dataclass
@dataclass
class person:
    name: str
    age: int
    email :str
if __name__ =="__main__":
    # 实例化对象，Python不需要new一块内存来存取对象
    p1=person("li",21,"2284610019@qq.com")
    p2=person("zhang",22,"yannianyishou1@gmail")
    # 内置的方法
    print(p1)
    # 内置的比较
    print(p1==p2)







