class vector3f{
  public:
  float x;
  float y;
  float z;
  vector3f operator =(float arg[3]){
    vector3f ret;
    ret.x = arg[0];
    ret.y = arg[1];
    ret.z = arg[2];
    return *ret;
  }
  vector3f operator =(vector3f& arg){
    vector3f ret;
    ret.x = arg.x;
    ret.y = arg.y;
    ret.z = arg.z;
    return *ret;
  }
  vector3f operator *(float arg){
    vector3f ret;
    ret.x *= arg;
    ret.y *= arg;
    ret.z *= arg;
    return *ret;
  }
  vector3f operator *(vector3f& arg){
    vector3f ret;
    ret.x *= arg.x;
    ret.y *= arg.y;
    ret.z *= arg.z;
    return *ret;
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
    return *ret;
  }
  vector3f operator +(vector3f& arg){
    vector3f ret;
    ret.x += arg.x;
    ret.y += arg.y;
    ret.z += arg.z;
    return *ret;
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
};

typedef struct{
  vector3f vector;
  unsigned long time;
}timed_vector_typedef;