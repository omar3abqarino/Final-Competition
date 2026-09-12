# 🌟 **How to Make a ROS2 Custom Service Type**

### 🧩 In your workspace:

1️⃣ You need to make a **CMake** package 📦 *(if yours is a Python package)*

**Command for creating a ROS2 CMake package in your workspace:**

```bash
cd ~/workspace/src
ros2 pkg create --build-type ament_cmake <package_name>
```

---

### 2️⃣ Create a separate folder for your `.srv` file 📁

In your new package terminal, type:

```bash
cd ~/workspace/src/<package_name>
mkdir srv
```

---

### 3️⃣ Define your service

Inside your `srv` folder, create a file named `service_name.srv`

```bash
cd srv
touch service_name.srv
code .
```

Define your service:

```
int64 a
int64 b
int64 c
---
int64 sum
```

➡️ This custom service requests three integers (**a**, **b**, **c**) and responds with one integer (**sum**).

---

### 4️⃣ Update your **CMakeLists.txt**

These lines will **auto-generate Python code** for your service definition so it can be used in Python 🚀

Add the following:

```cmake
find_package(rosidl_default_generators REQUIRED)
# find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/service_name.srv"
  # DEPENDENCIES std_msgs
)
```

📝 *Tip:* Uncomment or replace the hashed lines if you’re adding dependencies.
For example, if your response uses `sensor_msgs/Image`, you’ll need:

```cmake
find_package(sensor_msgs REQUIRED)
```

---

### 5️⃣ Add dependencies to your **package.xml**

Add these lines inside your XML file:

```xml
<!-- <depend>std_msgs</depend> -->

<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

🧠 The last three lines are **required** for packages that auto-generate Python code for your service.

---

### 6️⃣ **Build your package**

```bash
cd ~/workspace
colcon build --packages-select <package_name>
```

---

### 7️⃣ **Source your package**

```bash
. install/setup.bash
```

---

### 8️⃣ **Check that ROS2 sees your service**

```bash
ros2 interface show <package_name>/srv/service_name
```

✅ You should now see exactly what you defined in your `.srv` file!

---

### 9️⃣ **Use your service in Python** 🥳

**Server Node**

```python
from package_name.srv import service_type_name

create_service(service_type_name, 'service_name', call_back_function)
```

**Client Node**

```python
create_client(service_type_name, 'service_name')

request = service_type_name.Request()
```

---

### 🔟 Add your service as a dependency 📄

In any package that uses this service, update its `package.xml`:

```xml
<exec_depend>tutorial_interfaces</exec_depend>
```

---

## 📚 Helpful Resources

**ROS2 Docs:**
🧷 [Creating Custom ROS 2 msg and srv Files](https://docs.ros.org/en/crystal/Tutorials/Custom-ROS2-Interfaces.html)
🧷 [Writing a Simple Service and Client (Python)](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)


