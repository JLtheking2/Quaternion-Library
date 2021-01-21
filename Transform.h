#pragma once
#include "Component.h"
#include "Vector3D.h"
#include "ImGuiPropertyInspector.h"
#include "Subject.h"
#include "Quat.h"
#include "Matrix3x3.h"
#include "Matrix4x4.h"
#include <iostream>

namespace SNova
{
class Transform : public Component, public Subject
{
	friend struct Rotator;
	friend struct Quat;

public:
	Transform();
	Transform(const Transform& rhs);
	Transform& operator= (const Transform& rhs);

	// position
	void SetPosition(const SNova::Vec3& pos);
	void SetPosition(const float& x, const float& y, const float& z);
	void SetPosX(const float& x);
	void SetPosY(const float& y);
	void SetPosZ(const float& z);
	SNova::Vec3 GetPosition() const;
	const float& GetPosX() const;
	const float& GetPosY() const;
	const float& GetPosZ() const;

	// rotation
	const SNova::Matrix3x3& GetRotationMatrixInDegrees() const;
  
  //scale
	void SetScale(const SNova::Vec3& s);
	void SetScale(float x, float y, float z);
	void SetScale(float uniform);
	void SetScaleX(const float& x);
	void SetScaleY(const float& y);
	void SetScaleZ(const float& z);
	SNova::Vec3 GetScale() const;
	float GetScaleX() const;
	float GetScaleY() const;
	float GetScaleZ() const;

	//Mtx
	Mtx44 GetTransform() const { return mtx; };

	//Properties
	void ListProperties() override;

	static void* Create(void* object = nullptr);
	Transform* Clone() override;

	std::string GetName() const  override;
	void RemoveComponentButton() override;

	// ****************** TAG SYSTEM ******************
	// For simplicity, we delimit multiple tags with whitespace
	// e.g. "NoSave Plant" contains tags "NoSave" and "Plant"

	// Naive Getters/Setters. Use next 3 functions for gameplay
	std::string GetTag() const;
	void SetTag(const std::string& tags);

	// Returns true if tag is present
	bool HasTag(const std::string& tag);
	void AddTag(const std::string& tag);
	void RemoveTag(const std::string& tag);

	void Serialize(std::ofstream& file) override;
	void Deserialize(std::ifstream& file, std::string input) override;

	property_vtable();
	Quat rotation;		// internal only
	Rotator rotator;	// exposed to editor

	//Mtx44 GetRotationMtx();

private:
	SNova::Vec3 position;
	SNova::Vec3 scale;
	std::string Tag = std::string{};

	//Transform matrix 
	Mtx44 mtx;

	void UpdateMtx();
};

}

property_begin_name(SNova::Transform, "Transform")
{
	property_parent(Component)
	, property_var(Tag)
	, property_var_fnbegin("position", SNova::Vec3)
	{
		if (isRead)
		{
			InOut = Self.position;
		}
		else
		{
			Self.SetPosition(InOut);
		}

	}property_var_fnend()

	, property_var_fnbegin("scale", SNova::Vec3)
	{
		if (isRead)
		{
			InOut = Self.scale;
		}
		else
		{
			Self.SetScale(InOut);
		}

	}property_var_fnend()

	, property_var(rotation.x).Flags(property::flags::SHOW_READONLY)
	, property_var(rotation.y).Flags(property::flags::SHOW_READONLY)
	, property_var(rotation.z).Flags(property::flags::SHOW_READONLY)
	, property_var(rotation.w).Flags(property::flags::SHOW_READONLY)
	, property_var(rotator)
	// no longer neeeded due to quat being bound to rotator
	//, property_var_fnbegin("rotation", SNova::Rotator)
	//		{
	//			if (isRead)
	//			{
	//				InOut = Self.rotator;
	//			}
	//			else
	//			{
	//				Self.rotator = InOut;
	//				Self.UpdateMtx();
	//				//Self.rotation = Self.rotator.Quaternion();
	//			}
	//		}property_var_fnend()
}
property_vend_h(SNova::Transform)

