class vector3f{
  public:
  float x;
  float y;
  float z;
  vector3f operator =(float arg[3]){
    this->x = arg[0];
    this->y = arg[1];
    this->z = arg[2];
    return *this;
  }
  vector3f operator *(float arg){
    vector3f ret;
    ret.x *= arg;
    ret.y *= arg;
    ret.z *= arg;
    return ret;
  }
  vector3f operator *(vector3f& arg){
    vector3f ret;
    ret.x *= arg.x;
    ret.y *= arg.y;
    ret.z *= arg.z;
    return ret;
  }
  vector3f operator *=(float arg){
    this->x *= arg;
    this->y *= arg;
    this->z *= arg;
    return *this;
  }
  vector3f operator *=(vector3f& arg){
    this->x *= arg.x;
    this->y *= arg.y;
    this->z *= arg.z;
    return *this;
  }
  vector3f operator +(float arg){
    vector3f ret;
    ret.x += arg;
    ret.y += arg;
    ret.z += arg;
    return ret;
  }
  vector3f operator +(vector3f& arg){
    vector3f ret;
    ret.x += arg.x;
    ret.y += arg.y;
    ret.z += arg.z;
    return ret;
  }
  vector3f operator +=(float arg){
    this->x += arg;
    this->y += arg;
    this->z += arg;
    return *this;
  }
  vector3f operator +=(vector3f& arg){
    this->x += arg.x;
    this->y += arg.y;
    this->z += arg.z;
    return *this;
  }
  vector3f operator -(float arg){
    vector3f ret;
    ret.x -= arg;
    ret.y -= arg;
    ret.z -= arg;
    return ret;
  }
  vector3f operator -(vector3f& arg){
    vector3f ret;
    ret.x -= arg.x;
    ret.y -= arg.y;
    ret.z -= arg.z;
    return ret;
  }
  vector3f operator -=(float arg){
    this->x -= arg;
    this->y -= arg;
    this->z -= arg;
    return *this;
  }
  vector3f operator -=(vector3f& arg){
    this->x -= arg.x;
    this->y -= arg.y;
    this->z -= arg.z;
    return *this;
  }
  float operator [](int arg){
    switch (arg)
    {
    case 0:
      return this->x;
    case 1:
      return this->y;
    case 2:
      return this->z;    
    default:
      return 0.0;
    }
  }
};

typedef struct{
  vector3f vector;
  unsigned long time;
}timed_vector_typedef;