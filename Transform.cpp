#include "SNova.h"
#include "Factory.h"
#include "imgui.h"
//#include "OpenGLSystem.h"
#include "ReflectionDrawFns.h"
#include "Rotator.h"
#include <fstream>

namespace SNova
{

	void Transform::UpdateMtx()
	{
		// create rotation matrix
		Mtx44 rotMtx{ rotator.Matrix() };
		// create scaling matrix
		Mtx44 scaleMtx;
		Mtx44Scale(scaleMtx, scale.x, scale.y, scale.z);
		// create translation matrix
		Mtx44 transMtx;
		Mtx44Translate(transMtx, position.x, position.y, position.z);

		// concatenate the matrices
		mtx = transMtx * (rotMtx * scaleMtx);

		// notify observers regarding change of mtx
		Notify();
	}

	Transform::Transform()
		: scale{ 1.0f, 1.0f, 1.0f }	// default size
	{
		//OpenGLSystem::Get().Subscribe(this);

		// Bind Quat and Rotator together
		rotator.mp_BoundTransform = this;
		rotation.mp_BoundTransform = this;

		UpdateMtx();
	}

	Transform::Transform(const Transform& rhs)
	: position{ rhs.position }
	, scale{ rhs.scale }
	, rotation{ rhs.rotation }
	, rotator{ rhs.rotation }
	{
		// Bind Quat and Rotator together
		rotator.mp_BoundTransform = this;
		rotation.mp_BoundTransform = this;
		UpdateMtx();
	}

	Transform& Transform::operator=(const Transform& rhs)
	{
		Component::operator=(rhs);
		SetPosition(rhs.position);
		SetScale(rhs.scale);
		rotation = rhs.rotation;
		rotator = rhs.rotator;

		// Bind Quat and Rotator together
		rotator.mp_BoundTransform = this;
		rotation.mp_BoundTransform = this;

		UpdateMtx();
		return *this;
	}

	void Transform::SetPosition(const Vec3& pos)
	{
		position = pos;
		UpdateMtx();
	}

	void Transform::SetPosition(const float& x, const float& y, const float& z)
	{
		position.x = x;
		position.y = y;
		position.z = z;
		UpdateMtx();
	}

	void Transform::SetPosX(const float& x)
	{
		position.x = x;
		UpdateMtx();
	}

	void Transform::SetPosY(const float& y)
	{
		position.y = y;
		UpdateMtx();
	}

	void Transform::SetPosZ(const float& z)
	{
		position.z = z;
		UpdateMtx();
	}

	Vec3 Transform::GetPosition() const
	{
		return position;
	}

	const float& Transform::GetPosX() const
	{
		return position.x;
	}
	const float& Transform::GetPosY() const
	{
		return position.y;
	}
	const float& Transform::GetPosZ() const
	{
		return position.z;
	}
	
	const SNova::Matrix3x3& Transform::GetRotationMatrixInDegrees() const
	{
		float roX = rotator.pitch * 0.01745328888f;
		float roY = rotator.yaw * 0.01745328888f;
		float roZ = rotator.roll * 0.01745328888f;

		Matrix3x3 rotX{ 1.0f, 0, 0,
						0, cosf(roX), -sinf(roX),
						0, sinf(roX), cosf(roX) };

		Matrix3x3 rotY{ cosf(roY), 0, sinf(roY),
						0, 1.0f, 0
						-sinf(roY), 0, cosf(roY) };

		Matrix3x3 rotZ{ cosf(roZ), -sinf(roZ), 0,
						sinf(roZ), cosf(roZ), 0,
						0, 0, 1.0f };

		return rotZ * rotY * rotX;
	}

	void Transform::SetScale(const Vec3& s)
	{
		scale = s;
		UpdateMtx();
	}

	void Transform::SetScaleX(const float& x)
	{
		scale.x = x;
		UpdateMtx();
	}

	void Transform::SetScaleY(const float& y)
	{
		scale.y = y;
		UpdateMtx();
	}

	void Transform::SetScaleZ(const float& z)
	{
		scale.z = z;
		UpdateMtx();
	}

	void Transform::SetScale(float x, float y, float z)
	{
		scale = Vec3{ x, y, z };
		UpdateMtx();
	}

	void Transform::SetScale(float uniform)
	{
		scale = Vec3{ uniform, uniform, uniform };
		UpdateMtx();
	}

	Vec3 Transform::GetScale() const
	{
		return scale;
	}

	float Transform::GetScaleX() const
	{
		return scale.x;
	}

	float Transform::GetScaleY() const
	{
		return scale.y;
	}

	float Transform::GetScaleZ() const
	{
		return scale.z;
	}

	std::string Transform::GetTag() const
	{
		return Tag;
	}

	void Transform::SetTag(const std::string& input)
	{
		Tag = input;
	}

	// Helper function for tag system
	std::vector<std::string> SplitTags(const std::string& tags)
	{
		std::istringstream iss(tags);
		std::vector<std::string> result{
			std::istream_iterator<std::string>(iss), {}
		};
		return result;
	}

	// Helper function for tag system
	std::string ToLower(const std::string str)
	{
		std::string tmp{ str };
		std::transform(tmp.begin(), tmp.end(), tmp.begin(),
			[](unsigned char c) { return std::tolower(c); });
		return tmp;
	}

	bool Transform::HasTag(const std::string& target)
	{
		return Tag.find(target) != std::string::npos;
	}

	void Transform::AddTag(const std::string& newTag)
	{
		if (Tag == "")
			SetTag(newTag);

		if (!HasTag(newTag))
			SetTag(Tag + ' ' + newTag);
	}

	void Transform::RemoveTag(const std::string& target)
	{
		auto pos = Tag.find(target);
		auto len = target.length();

		if (pos == std::string::npos)
			return;

		Tag.replace(pos, len+1, "");

		if (!Tag.empty() && Tag.back() == ' ')
			Tag.pop_back();
	}

	void Transform::ListProperties()
	{
		if (ImGui::CollapsingHeader(GetName().c_str()))
		{
			//ImGui::Text("Modify through LIONANT Reflection");
			REFLECTION::RenderProperty(*this);

			//Old Implementation
			/*ImGui::Separator();
			ImGui::Separator();
			
			ImGui::Text("Modify through our virtual function");
			ImGui::Text("Position");

			static float size = 60.0f;

			ImGui::SetNextItemWidth(size);
			ImGui::DragFloat("x pos", &position.x, 0.1f);

			SameLineWidget(size);
			ImGui::DragFloat("y pos", &position.y, 0.1f);

			SameLineWidget(size);
			ImGui::DragFloat("z pos", &position.z, 0.1f);

			ImGui::Separator();

			ImGui::Text("Scale");

			ImGui::SetNextItemWidth(size);
			ImGui::DragFloat("x scl", &scale.x, 0.1f);

			SameLineWidget(size);
			ImGui::DragFloat("y scl", &scale.y, 0.1f);

			SameLineWidget(size);
			ImGui::DragFloat("z scl", &scale.z, 0.1f);

			ImGui::Separator();

			ImGui::Text("Rotation");

			ImGui::SetNextItemWidth(size);
			ImGui::DragFloat("x rot", &rotation.x);

			SameLineWidget(size);
			ImGui::DragFloat("y rot", &rotation.y);

			SameLineWidget(size);
			ImGui::DragFloat("z rot", &rotation.z);

			ImGui::Separator();*/
		}
	}

	void* Transform::Create(void* object)
	{
		char* tmp = (char*)MemoryManager::Allocate(sizeof(Transform), MemoryManager::TRANSFORM);
		new (tmp) Transform{};

		return tmp;
	}

	Transform* Transform::Clone()
	{
		Transform* newCmp = (Transform*)FACTORY::Factory::Get().CreateObject("Transform");

		newCmp->SetPosition(this->GetPosition());
		newCmp->SetScale(this->GetScale());

		return newCmp;
	}

	std::string Transform::GetName() const
	{
		return std::string{"Transform"};
	}
	void Transform::RemoveComponentButton()
	{

	}
	void Transform::Serialize(std::ofstream& file)
	{
		file << "[TRANSFORM]";
		file << "\n";
		file << "TAG:" << Tag;
		file << "\n";
		file << "POSITION:" << position.x << "," << position.y << "," << position.z;
		file << "\n";
		file << "SCALE:" << scale.x << "," << scale.y << "," << scale.z;
		file << "\n";
		file << "ROTATION: " << rotator.pitch << " " << rotator.yaw << " " << rotator.roll;
		file << "\n";
		Component::Serialize(file);
	}
	void Transform::Deserialize(std::ifstream& file, std::string input)
	{
		std::getline(file, input); // Tag
		std::string _str = input.substr(input.find(":") + 1);
		Tag = _str;

		std::getline(file, input); // position
		input = input.substr(input.find(":") + 1);
		std::istringstream is(input);

		std::getline(is, input, ',');
		position.x = std::stof(input);
		std::getline(is, input, ',');
		position.y = std::stof(input);
		std::getline(is, input, ',');
		position.z = std::stof(input);

		std::getline(file, input); // scale
		input = input.substr(input.find(":") + 1);
		is.clear();
		is.str(input);

		std::getline(is, input, ',');
		scale.x = std::stof(input);
		std::getline(is, input, ',');
		scale.y = std::stof(input);
		std::getline(is, input, ',');
		scale.z = std::stof(input);

		// rotation
		// With safety check for old savefiles
		auto t = file.tellg(); 

		std::getline(file, input);
		is.clear();
		is.str(input);

		std::string tmp;
		is >> tmp;
		if (tmp != "ROTATION:")
		{
			// Legacy save file without rotation
			// Rewind and end deserialisation
			file.seekg(t);
			return;
		}
		is >> rotator.pitch >> rotator.yaw >> rotator.roll;
		rotator.UpdateBoundTransform();
		UpdateMtx();

		Component::Deserialize(file, input);
	}

	/*Mtx44 Transform::GetRotationMtx()
	{
		Mtx44 xRot;
		Mtx44 yRot;
		Mtx44 zRot;
		void Mtx44RotXDeg(xRot, float angle);
		void Mtx44RotYDeg(yRot, float angle);
		void Mtx44RotZDeg(zRot, float angle);
	}*/
}

