from __future__ import annotations
from math import cos,sin

class Vec3:
    x:float
    y:float
    z:float
    def __init__(self,x:float|int=0,y:float|int|None=None,z:float|int|None=None) -> None:
        if x is not None and y is None and z is None:
            y=x
            z=x
        self.x=x
        self.y=y
        self.z=z
        
    
    @property
    def sqr_magnitude(self)->float:
        return (self.x**2+self.y**2+self.z**2)
    
    @property
    def magnitude(self)->float:
        return self.sqr_magnitude**0.5
    @property
    def normalised(self)->Vec3:
        return self/self.magnitude
    def dot(self,other:Vec3)->float:
        result = self.x*other.x+self.y*other.y+self.z*other.z
        return result
    @classmethod
    def from_angle(cls,angle:float)->Vec3:
        return Vec3(cos(angle),sin(angle),0)
    # unary operators
    def __pos__(self)->Vec3:
        return self
    def __neg__(self)->Vec3:
        return Vec3(-self.x,-self.y,-self.z)
    
    # binary operators
    def __add__(self,other:Vec3|float|int)->Vec3:
        if type(other) is not Vec3:
            other=Vec3(other)
        return Vec3(self.x+other.x,self.y+other.y,self.z+other.z)
    def __radd__(self,other:float|int)->Vec3:
        return self+other
    def __sub__(self,other:Vec3|float|int)->Vec3:
        return self + -other
    def __mul__(self,other:Vec3|float|int)->Vec3:
        if type(other) is not Vec3:
            other=Vec3(other)
        return Vec3(self.x*other.x,self.y*other.y,self.z*other.z)
    def __rmul__(self, other:float|int)->Vec3:
        return self*other
    def __truediv__(self,other:Vec3|float|int)->Vec3:
        if type(other) is not Vec3:
            other=Vec3(other)
        return Vec3(self.x/other.x,self.y/other.y,self.z/other.z)
    def __floordiv__(self,other:float|int)->Vec3:
        return Vec3(self.x//other,self.y//other,self.z//other)
    def __mod__(self,other:Vec3|float|int)->Vec3:
        if type(other) is not Vec3:
            other=Vec3(other)
        return Vec3(self.x%other.x,self.y%other.y,self.z%other.z)

    
    # comparison operators
    def __eq__(self,other:Vec3)->bool:
        return self.x==other.x and self.y==other.y and self.z==other.z
    def __ne__(self,other:Vec3)->bool:
        return not self == other

    def __str__(self)->str:
        return f"Vec3(x={self.x}, y={self.y}, z={self.z})"
    def __repr__(self)->str:
        return str(self)
    def __hash__(self):
        return hash(self.x)^hash(self.y)^hash(self.z)
    def __dict__(self)->dict:
        return {"x":self.x,
                "y":self.y,
                "z":self.z}
