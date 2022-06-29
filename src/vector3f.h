class vector3f{
  public:
  float x = 0;
  float y = 0;
  float z = 0;
  vector3f(float x, float y, float z){
    this->x = x;
    this->y = y;
    this->z = z;
  };
  vector3f operator =(float arg[3]){
    this->x = arg[0];
    this->y = arg[1];
    this->z = arg[2];
    return *this;
  }
  vector3f operator *(float arg){
    vector3f ret = {this->x * arg, this->y * arg, this->z * arg};
    return ret;
  }
  vector3f operator *(float arg[3]){
    vector3f ret = {this->x * arg[0], this->y * arg[1], this->z * arg[2]};
    return ret;
  }
  vector3f operator *(vector3f& arg){
    vector3f ret = {this->x * arg.x, this->y * arg.y, this->z * arg.z};
    return ret;
  }
  vector3f operator *=(float arg){
    this->x *= arg;
    this->y *= arg;
    this->z *= arg;
    return *this;
  }
  vector3f operator *=(float arg[3]){
    this->x *= arg[0];
    this->y *= arg[1];
    this->z *= arg[2];
    return *this;
  }
  vector3f operator *=(vector3f& arg){
    this->x *= arg.x;
    this->y *= arg.y;
    this->z *= arg.z;
    return *this;
  }
  vector3f operator +(float arg){
    vector3f ret = {this->x + arg, this->y + arg, this->z + arg};
    return ret;
  }
  vector3f operator +(float arg[3]){
    vector3f ret = {this->x + arg[0], this->y + arg[1], this->z + arg[2]};
    return ret;
  }
  vector3f operator +(vector3f& arg){
    vector3f ret = {this->x + arg.x, this->y + arg.y, this->z + arg.z};
    return ret;
  }
  vector3f operator +=(float arg){
    this->x += arg;
    this->y += arg;
    this->z += arg;
    return *this;
  }
  vector3f operator +=(float arg[3]){
    this->x += arg[0];
    this->y += arg[1];
    this->z += arg[2];
    return *this;
  }
  vector3f operator +=(vector3f& arg){
    this->x += arg.x;
    this->y += arg.y;
    this->z += arg.z;
    return *this;
  }
  vector3f operator -(float arg){
    vector3f ret = {this->x - arg, this->y - arg, this->z - arg};
    return ret;
  }
  vector3f operator -(float arg[3]){
    vector3f ret = {this->x - arg[0], this->y - arg[1], this->z - arg[2]};
    return ret;
  }
  vector3f operator -(vector3f& arg){
    vector3f ret = {this->x - arg.x, this->y - arg.y, this->z - arg.z};
    return ret;
  }
  vector3f operator -=(float arg){
    this->x -= arg;
    this->y -= arg;
    this->z -= arg;
    return *this;
  }
  vector3f operator -=(float arg[3]){
    this->x -= arg[0];
    this->y -= arg[1];
    this->z -= arg[2];
    return *this;
  }
  vector3f operator -=(vector3f& arg){
    this->x -= arg.x;
    this->y -= arg.y;
    this->z -= arg.z;
    return *this;
  }
  float* operator [](int arg){
    switch (arg)
    {
    case 0:
      return &this->x;
    case 1:
      return &this->y;
    case 2:
      return &this->z;    
    default:
      return nullptr;
    }
  }
};

class matrix3f{
  public:
  vector3f row[3] {vector3f(0,0,0), vector3f(0,0,0), vector3f(0,0,0)};
  matrix3f(vector3f row0, vector3f row1, vector3f row2){
    this->row[0] = row0;
    this->row[1] = row1;
    this->row[2] = row2;
  }
  matrix3f(vector3f row[3]){
    this->row[0] = row[0];
    this->row[1] = row[1];
    this->row[2] = row[2];
  }
  matrix3f(){
    this->row[0] = vector3f(0,0,0);
    this->row[1] = vector3f(0,0,0);
    this->row[2] = vector3f(0,0,0);
  }
  matrix3f operator =(float arg[3][3]){
    for (int i = 0; i < 3; i++)
    {
      this->row[i] = arg[i];
    }
    return *this;
  }
  matrix3f operator *(float arg){
    matrix3f ret{this->row[0] * arg, this->row[1] * arg, this->row[2] * arg};
    return ret;
  }
  friend vector3f operator *(matrix3f mat, vector3f vec){
    vector3f ret{0,0,0};
    for (int i = 0; i < 3; i++)
    {
      for(int j = 0; j < 3; j++){
        *ret[i] += *(mat.row[i])[j] * *vec[j];
      }
    }
    return ret;
  }
};
typedef struct{
  vector3f vector = {0,0,0};
  unsigned long time;
}timed_vector_typedef;