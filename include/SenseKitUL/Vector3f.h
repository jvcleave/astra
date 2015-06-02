#ifndef VECTOR3F_H
#define VECTOR3F_H

#include <math.h>
#include <SenseKitUL/skul_ctypes.h>

namespace sensekit {

    struct Vector3f : public sensekit_vector3f_t
    {
        static inline Vector3f& from_cvector(sensekit_vector3f_t& cvector)
        {
            return *reinterpret_cast<Vector3f*>(&cvector);
        }

        static inline const Vector3f& from_cvector(const sensekit_vector3f_t& cvector)
        {
            return *reinterpret_cast<const Vector3f*>(&cvector);
        }

        static inline Vector3f* from_cvector_ptr(sensekit_vector3f_t* p_cvector)
        {
            return reinterpret_cast<Vector3f*>(p_cvector);
        }

        Vector3f()
        {
            this->x = 0;
            this->y = 0;
            this->z = 0;
        }

        Vector3f(float x, float y, float z)
        {
            this->x = x;
            this->y = y;
            this->z = z;
        }

        float length() const;
        float length_squared() const;
        float dot(const Vector3f& v) const;

        Vector3f cross(const Vector3f& v) const;

        static inline Vector3f zero();
        static Vector3f normalize(Vector3f v);

        friend bool operator==(const Vector3f& lhs, const Vector3f& rhs);
        friend bool operator!=(const Vector3f& lhs, const Vector3f& rhs);

        bool is_zero() const;

        Vector3f& operator+=(const Vector3f& rhs);
        Vector3f& operator-=(const Vector3f& rhs);
        Vector3f& operator*=(const float& rhs);
        Vector3f& operator/=(const float& rhs);

        friend Vector3f operator+(const Vector3f& lhs, const Vector3f& rhs);
        friend Vector3f operator-(const Vector3f& lhs, const Vector3f& rhs);
        friend Vector3f operator*(const Vector3f& lhs, const float& rhs);
        friend Vector3f operator*(const float& lhs, const Vector3f& rhs);
        friend Vector3f operator/(const Vector3f& lhs, const float& rhs);

        Vector3f operator-();
    };

    inline float Vector3f::length() const
    {
        return sqrtf(x * x + y * y + z * z);
    }

    inline float Vector3f::length_squared() const
    {
        return x * x + y * y + z * z;
    }

    inline float Vector3f::dot(const Vector3f& v) const
    {
        return x * v.x + y * v.y + z * v.z;
    }

    inline Vector3f Vector3f::cross(const Vector3f& v) const
    {
        return Vector3f(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
    }

    inline Vector3f Vector3f::normalize(Vector3f v)
    {
        double length = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
        if (length < 1e-9)
        {
            return Vector3f(0, 0, 0);
        }
        else
        {
            return Vector3f(v.x / length, v.y / length, v.z / length);
        }
    }

    inline Vector3f Vector3f::zero()
    {
        static Vector3f zero;
        return zero;
    }

    inline bool Vector3f::is_zero() const
    {
        return *this == zero();
    }

    inline Vector3f& Vector3f::operator+=(const Vector3f& rhs)
    {
        this->x = this->x + rhs.x;
        this->y = this->y + rhs.y;
        this->z = this->z + rhs.z;
        return *this;
    }

    inline Vector3f& Vector3f::operator-=(const Vector3f& rhs)
    {
        this->x = this->x - rhs.x;
        this->y = this->y - rhs.y;
        this->z = this->z - rhs.z;
        return *this;
    }

    inline Vector3f& Vector3f::operator*=(const float& rhs)
    {
        this->x = this->x * rhs;
        this->y = this->y * rhs;
        this->z = this->z * rhs;
        return *this;
    }

    inline Vector3f& Vector3f::operator/=(const float& rhs)
    {
        this->x = this->x / rhs;
        this->y = this->y / rhs;
        this->z = this->z / rhs;
        return *this;
    }

    inline Vector3f Vector3f::operator-()
    {
        return Vector3f(-this->x, -this->y, -this->z);
    }

    inline bool operator==(const Vector3f& lhs, const Vector3f& rhs)
    {
        return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
    }

    inline bool operator!=(const Vector3f& lhs, const Vector3f& rhs)
    {
        return !(lhs == rhs);
    }

    inline Vector3f operator+(const Vector3f& lhs, const Vector3f& rhs)
    {
        return Vector3f(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
    }

    inline Vector3f operator-(const Vector3f& lhs, const Vector3f& rhs)
    {
        return Vector3f(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
    }

    inline Vector3f operator*(const Vector3f& lhs, const float& rhs)
    {
        return Vector3f(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
    }

    inline Vector3f operator*(const float& lhs, const Vector3f& rhs)
    {
        return rhs * lhs;
    }

    inline Vector3f operator/(const Vector3f& lhs, const float& rhs)
    {
        return Vector3f(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs);
    }
}

#endif /* VECTOR3F_H */