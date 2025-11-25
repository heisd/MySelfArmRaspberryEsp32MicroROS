class person:
    def __init__(self,name,age,email):
        self.name=name
        self.age=age
        self.email=email
    def __eq__(self,other):
        return self.name == other.name and self.age == other.age and self.email== other.email
    def __repr__(self):
        return f"person({self.name},{self.age},{self.email})"
if __name__ =="__main__":
    p1=person("li",21,"2284610019")
    p2=person("zhang",22,"yannianyishou1@gmail.com")
    print(p1)
    print(p1==p2)
