#include "Helpers.h"
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <utility>
#include <set>

using namespace Eigen;
/********Gloable Variable**********/
VertexBufferObject VBO;
VertexBufferObject VBO_normal;
Eigen::MatrixXf V(3, 3);
Eigen::MatrixXf N(3, 3);
Eigen::MatrixXf N_flat;
//useful matrix
Eigen::Matrix4f view(4, 4);
Eigen::Matrix4f model(4, 4);
Eigen::Matrix4f projection(4, 4);

Eigen::Matrix4f original_view(4, 4);
Eigen::Matrix4f original_model(4, 4);
Eigen::Matrix4f original_projection(4, 4);

//Set for V and F
std::vector<Eigen::MatrixXd> V_set;
std::vector<Eigen::MatrixXi> F_set;
//the position of cube, bumpy and bunny in the whole set of objects
std::vector<int> cube_pos;
std::vector<int> bumpy_pos;
std::vector<int> bunny_pos;
std::vector<int> test_pos;
//total number of object
int all_object = 0;
//each number of three objects
int cube_no = 0;
int bunny_no = 0;
int bumpy_no = 0;
int test_no = 0;
//the begin index for each object
int size_array[100] = {};
//object size
const int cube_size = 36;
const int bunny_size = 3000;
const int bumpy_size = 3000;
const int test_size = 3;
//View mode
Eigen::Matrix4f orthograohic(4, 4);
Eigen::Matrix4f perspective(4, 4);
const double pi = 3.1415926;
char view_mode = 'p';
char show_mode = 'n';
//camera
double distance_r = 5;
double angle_theta = 90;
double angle_phi = 0;
float camera_x;
float camera_y;
float camera_z;
//phong
struct phong {
	Eigen::MatrixXi F_off;
	int area[1000];//F->area;
	std::vector< std::pair<Eigen::Vector3d, double> > v_normal[502];//Vertex-->normal
	Eigen::Vector3d V_normal_average[502];

};
bool cube_phong_flag = false;
bool bumpy_phong_flag = false;
bool bunny_phong_flag = false;

phong Sphong[3];
//selection
float near_z;
double mouse_x;
double mouse_y;
bool select_flag = false;
int object_pos = -1;
Eigen::MatrixXf V_changed;

/********Function Declare**********/
void init_matrix(GLFWwindow* window);
void read_file(std::string filename);
void update_to_VN(std::string filename);
void add_cube();
void add_bumpy();
void add_bunny();
void add_test();
void draw_object(Program& program);
void draw_cube(Program& program);
void draw_bumpy(Program& program);
void draw_bunny(Program& program);
void angle_change();
void rotate(char x, double angle);
void scale(char flag);
void move(char flag);

void scale_v(char flag);
void rotate_v(char flag);
void move_v(char flag);

Eigen::Matrix4f InitCameraTransform(const Eigen::Vector3f Target, const Eigen::Vector3f Up);
void generate_phong(Eigen::MatrixXd& V_off, Eigen::MatrixXi& F_off, std::string filename);
double triangle_area(Eigen::Vector3d point1, Eigen::Vector3d point2, Eigen::Vector3d point3);
Eigen::Vector3d triangle_normal(Eigen::Vector3d point1, Eigen::Vector3d point2, Eigen::Vector3d point3);
void calculate_average(std::string filename);
void update_N(std::string filename, int start_pos);


int get_object_pos();
void update_v_changed(std::string filename, int start_point, Matrix4f model, Matrix4f projecttion, Matrix4f view);
bool mouse_in_object(int begin, int end, Vector3d ray, Vector3d view_position);
bool object_on_plane(int begin, int end, Vector3d point, Vector3d normal);
Vector2d point_projection_o(Vector3d point, Vector3d point_plane, Vector3d normal);
Vector2d point_projection_p(Vector3d point, Vector3d point_plane, Vector3d normal);
float sign(double p1_x, double p1_y, double p2_x, double p2_y, double p3_x, double p3_y);
bool PointInTriangle(double pt_x, double pt_y, double v1_x, double v1_y, double v2_x, double v2_y, double v3_x, double v3_y);
void delete_object();

/**********Implementation**********/
void init_matrix(GLFWwindow* window) {

	int width, height;
	glfwGetWindowSize(window, &width, &height);
	float aspect_ratio = float(height) / float(width);

	original_view << aspect_ratio, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	view = original_view;


	original_projection << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	projection = original_projection;


	original_model << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	model = original_model;

	float tan_alpha = tan(22.5 * pi / 180.0);

	near_z = 1;
	float far_z = 100;

	orthograohic << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, -distance_r,
		0, 0, 0, 1;

	perspective << 1.0f / (aspect_ratio * tan_alpha), 0, 0, 0,
		0, 1.0f / tan_alpha, 0, 0,
		0, 0, (-near_z - far_z) / (near_z - far_z), (2 * far_z * near_z) / (near_z - far_z),
		0, 0, 1, 0;
}
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	// Update the position of the first vertex if the keys 1,2, or 3 are pressed
	switch (key)
	{
	case  GLFW_KEY_1:
		if (action == GLFW_PRESS)
			add_cube();
		break;
	case  GLFW_KEY_2:
		if (action == GLFW_PRESS)
			add_bumpy();
		break;
	case  GLFW_KEY_3:
		if (action == GLFW_PRESS)
			add_bunny();
		break;
	case  GLFW_KEY_4:
		if (action == GLFW_PRESS)
			add_test();
		break;
	case  GLFW_KEY_O:
		if (action == GLFW_PRESS)
			view_mode = 'o';
		break;
	case  GLFW_KEY_P:
		if (action == GLFW_PRESS)
			view_mode = 'p';
		break;
	case  GLFW_KEY_I:
		if (action == GLFW_PRESS) {
			distance_r -= 1;
			angle_change();
		}
		break;
	case  GLFW_KEY_K:
		if (action == GLFW_PRESS) {
			distance_r += 1;
			angle_change();
		}
		break;
	case  GLFW_KEY_W:
		if (action == GLFW_PRESS) {
			angle_theta -= 10;
			angle_change();
		}
		break;
	case  GLFW_KEY_S:
		if (action == GLFW_PRESS) {
			angle_theta += 10;
			angle_change();
		}
		break;
	case  GLFW_KEY_A:
		if (action == GLFW_PRESS) {
			angle_phi -= 10;
			angle_change();
		}
		break;
	case  GLFW_KEY_D:
		if (action == GLFW_PRESS) {
			angle_phi += 10;
			angle_change();
		}
		break;
	case  GLFW_KEY_B:
		if (action == GLFW_PRESS) {
			show_mode = 'b';
		}
		break;
	case  GLFW_KEY_N:
		if (action == GLFW_PRESS) {
			show_mode = 'n';
		}
		break;

	case  GLFW_KEY_M:
		if (action == GLFW_PRESS) {
			show_mode = 'm';
		}
		break;

	case  GLFW_KEY_L:
		if (action == GLFW_PRESS) {
			rotate_v('y');
		}
		break;

	case  GLFW_KEY_MINUS:
		if (action == GLFW_PRESS) {
			scale_v('-');
		}
		break;
	case  GLFW_KEY_EQUAL:
		if (action == GLFW_PRESS) {
			scale_v('+');
		}
		break;

	case  GLFW_KEY_UP:
		if (action == GLFW_PRESS) {
			move_v('w');
		}
		break;
	case  GLFW_KEY_DOWN:
		if (action == GLFW_PRESS) {
			move_v('s');
		}
		break;
	case  GLFW_KEY_LEFT:
		if (action == GLFW_PRESS) {
			move_v('a');
		}
		break;

	case  GLFW_KEY_RIGHT:
		if (action == GLFW_PRESS) {
			move_v('d');
		}
		break;

	case  GLFW_KEY_BACKSPACE:
		if (action == GLFW_PRESS) {
			delete_object();
		}
		break;
	default:
		break;
	}

	// Upload the change to the GPU
	VBO.update(V);
	VBO_normal.update(N);
}

void add_cube() {
	read_file("cube");
	cube_pos.push_back(all_object);
	cube_no++;
	all_object++;
	size_array[all_object] = size_array[all_object - 1] + cube_size;
}
void add_bumpy() {
	read_file("bumpy");
	bumpy_pos.push_back(all_object);
	bumpy_no++;
	all_object++;
	size_array[all_object] = size_array[all_object - 1] + bumpy_size;
}
void add_bunny() {
	read_file("bunny");
	bunny_pos.push_back(all_object);
	bunny_no++;
	all_object++;
	size_array[all_object] = size_array[all_object - 1] + bunny_size;
}
void add_test() {
	//std::cout<<"adding test\n";
	read_file("test");
	test_pos.push_back(all_object);
	test_no++;
	all_object++;
	size_array[all_object] = size_array[all_object - 1] + test_size;
}

void read_file(std::string filename) {

	Eigen::MatrixXd V_off;
	Eigen::MatrixXi F_off;
	std::string line;
	std::string root_path = "../data/";
	std::string target_file;
	if (filename == "test")
		target_file = "test.off";
	else if (filename == "cube")
		target_file = "cube.off";
	else if (filename == "bumpy")
		target_file = "bumpy_cube.off";
	else if (filename == "bunny")
		target_file = "bunny.off";
	else
		return;

	std::ifstream meshfile(root_path + target_file);
	int iter = 0;
	int n_f = 0, n_v = 0;
	if (meshfile.is_open()) {
		while (getline(meshfile, line)) {
			if (iter == 1) {
				std::istringstream iss(line);
				iss >> n_v >> n_f;
				V_off.resize(n_v, 3);
				F_off.resize(n_f, 3);
			}
			if (iter > 1 && iter < n_v + 2) {
				std::istringstream iss(line);
				iss >> V_off(iter - 2, 0) >> V_off(iter - 2, 1) >> V_off(iter - 2, 2);
			}
			if (iter >= n_v + 2) {
				int temp;
				std::istringstream iss(line);
				iss >> temp >> F_off(iter - 2 - n_v, 0) >> F_off(iter - 2 - n_v, 1) >> F_off(iter - 2 - n_v, 2);
			}
			iter++;
		}
		meshfile.close();
	}
	else
		std::cout << "Uable to open file" << std::endl;

	F_set.push_back(F_off);
	V_set.push_back(V_off);
	update_to_VN(filename);
}
void update_to_VN(std::string filename) {

	int last_vector_index = F_set.size() - 1;
	int total_number_vertex = V.cols();
	int object_size = F_set[last_vector_index].size();
	V.conservativeResize(3, total_number_vertex + object_size);

	V_changed.conservativeResize(3, total_number_vertex + object_size);

	N.conservativeResize(3, total_number_vertex + object_size);
	int object_row = F_set[last_vector_index].rows();//each row is a triangle
	int object_col = F_set[last_vector_index].cols();//how many triangles you have
	for (int row = 0; row < object_row; row++) {
		for (int col = 0; col < object_col; col++) {
			int index_node = F_set[last_vector_index].row(row)[col];
			V.col(total_number_vertex + row * object_col + col) << V_set[last_vector_index].row(index_node)[0], V_set[last_vector_index].row(index_node)[1], V_set[last_vector_index].row(index_node)[2];
			V_changed.col(total_number_vertex + row * object_col + col) << V_set[last_vector_index].row(index_node)[0], V_set[last_vector_index].row(index_node)[1], V_set[last_vector_index].row(index_node)[2];
		}
	}


	//flat shade
	for (int i = 0; i < object_size / 3; i++) {
		Eigen::Vector3f p1 = Eigen::Vector3f(V.col(total_number_vertex + 3 * i)[0], V.col(total_number_vertex + 3 * i)[1], V.col(total_number_vertex + 3 * i)[2]);

		Eigen::Vector3f p2 = Eigen::Vector3f(V.col(total_number_vertex + 3 * i + 1)[0], V.col(total_number_vertex + 3 * i + 1)[1], V.col(total_number_vertex + 3 * i + 1)[2]);

		Eigen::Vector3f p3 = Eigen::Vector3f(V.col(total_number_vertex + 3 * i + 2)[0], V.col(total_number_vertex + 3 * i + 2)[1], V.col(total_number_vertex + 3 * i + 2)[2]);

		Eigen::Vector3f p12 = p2 - p1;
		Eigen::Vector3f p23 = p2 - p3;
		Eigen::Vector3f normal = p12.cross(p23);

		N.col(total_number_vertex + i * 3) = normal.transpose().normalized();
		N.col(total_number_vertex + i * 3 + 1) = normal.transpose().normalized();
		N.col(total_number_vertex + i * 3 + 2) = normal.transpose().normalized();
	}

	N_flat = N;
	//
	if (filename == "cube" && cube_phong_flag == false) {
		generate_phong(V_set[last_vector_index], F_set[last_vector_index], filename);
		cube_phong_flag = true;
	}
	if (filename == "bumpy" && bumpy_phong_flag == false) {
		generate_phong(V_set[last_vector_index], F_set[last_vector_index], filename);
		bumpy_phong_flag = true;
	}
	if (filename == "bunny" && bunny_phong_flag == false) {
		generate_phong(V_set[last_vector_index], F_set[last_vector_index], filename);
		bunny_phong_flag = true;
	}

	VBO.update(V);
	VBO_normal.update(N);
}

void draw_object(Program& program) {
	if (all_object != 0) {
		draw_cube(program);
		draw_bumpy(program);
		draw_bunny(program);
	}

}

void draw_cube(Program& program) {
	Eigen::Vector3f Target(camera_x, camera_y, camera_z);
	Eigen::Vector3f Up(0, 1, 0);
	Eigen::Matrix4f temp = InitCameraTransform(Target, Up);
	view = original_view * temp;
	if (view_mode == 'p') {
		projection = perspective;
	}
	if (view_mode == 'o') {
		projection = orthograohic;
	}
	for (int i = 0; i < cube_no; i++) {

		model = original_model;
		Eigen::Matrix4f scaler;
		scaler << 0.6, 0, 0, -1.5,
			0, 0.6, 0, 0,
			0, 0, 0.6, 0,
			0, 0, 0, 1;
		model = original_model * scaler;
		for (int j = 0; j < i; j++) {
			rotate('y', 20);
			rotate('z', 20);
			move('d');
			scale('-');
			scale('-');
		}

		glUniformMatrix4fv(program.uniform("model"), 1, GL_FALSE, model.data());
		glUniformMatrix4fv(program.uniform("view"), 1, GL_FALSE, view.data());
		glUniformMatrix4fv(program.uniform("projection"), 1, GL_FALSE, projection.data());

		int pos_objects = cube_pos[i];
		Matrix4f changed = model * view * projection;
		//update_v_changed("cube", pos_objects, changed);
		update_v_changed("cube", pos_objects, model, projection, view);

		if (show_mode == 'n') {
			N = N_flat;
			glDrawArrays(GL_TRIANGLES, size_array[pos_objects], cube_size);
		}
		if (show_mode == 'b') {
			for (int i = 0; i < cube_size / 3; i++) {
				glDrawArrays(GL_LINE_LOOP, size_array[pos_objects] + 3 * i, 3);
			}
		}
		if (show_mode == 'm') {
			update_N("cube", size_array[pos_objects]);
			glDrawArrays(GL_TRIANGLES, size_array[pos_objects], cube_size);
		}
	}
}
void draw_bumpy(Program& program) {

	Eigen::Vector3f Target(camera_x, camera_y, camera_z);
	Eigen::Vector3f Up(0, 1, 0);
	Eigen::Matrix4f temp = InitCameraTransform(Target, Up);
	view = original_view * temp;
	if (view_mode == 'p') {
		projection = perspective;
	}
	if (view_mode == 'o') {
		projection = orthograohic;
	}

	for (int i = 0; i < bumpy_no; i++) {

		model = original_model;
		Eigen::Matrix4f scaler;
		scaler << 0.1, 0, 0, -1.5,
			0, 0.1, 0, -1.5,
			0, 0, 0.1, 0,
			0, 0, 0, 1;
		model = original_model * scaler;
		for (int j = 0; j < i; j++) {
			rotate('z', 20);
			move('d');
			move('d');
			move('d');
			move('d');
			scale('-');
		}

		glUniformMatrix4fv(program.uniform("model"), 1, GL_FALSE, model.data());
		glUniformMatrix4fv(program.uniform("view"), 1, GL_FALSE, view.data());
		glUniformMatrix4fv(program.uniform("projection"), 1, GL_FALSE, projection.data());
		int pos_objects = bumpy_pos[i];
		//Matrix4f changed=model*view*projection;
		//update_v_changed("bumpy", pos_objects, changed);
		update_v_changed("bumpy", pos_objects, model, projection, view);
		if (show_mode == 'n') {
			N = N_flat;
			glDrawArrays(GL_TRIANGLES, size_array[pos_objects], bumpy_size);
		}
		if (show_mode == 'b') {
			for (int i = 0; i < bumpy_size / 3; i++) {
				glDrawArrays(GL_LINE_LOOP, size_array[pos_objects] + 3 * i, 3);
			}
		}
		if (show_mode == 'm') {
			update_N("bumpy", size_array[pos_objects]);
			glDrawArrays(GL_TRIANGLES, size_array[pos_objects], bumpy_size);
		}


	}
}
void draw_bunny(Program& program) {

	Eigen::Vector3f Target(camera_x, camera_y, camera_z);
	Eigen::Vector3f Up(0, -1, 0);
	Eigen::Matrix4f temp = InitCameraTransform(Target, Up);
	view = original_view * temp;
	if (view_mode == 'p') {
		projection = perspective;
	}
	if (view_mode == 'o') {
		projection = orthograohic;
	}

	for (int i = 0; i < bunny_no; i++) {

		model = original_model;
		Eigen::Matrix4f scaler;
		scaler << 5, 0, 0, 1.5,
			0, 5, 0, -2,
			0, 0, 5, 0,
			0, 0, 0, 1;

		model = original_model * scaler;

		for (int j = 0; j < i; j++) {
			rotate('y', 10);
			move('a');
			scale('-');
		}

		glUniformMatrix4fv(program.uniform("model"), 1, GL_FALSE, model.data());
		glUniformMatrix4fv(program.uniform("view"), 1, GL_FALSE, view.data());
		glUniformMatrix4fv(program.uniform("projection"), 1, GL_FALSE, projection.data());
		int pos_objects = bunny_pos[i];
		//Matrix4d changed=model*view*projection;
		//update_v_changed("bunny", pos_objects, changed);
		update_v_changed("bunny", pos_objects, model, projection, view);
		if (show_mode == 'n') {
			N = N_flat;
			glDrawArrays(GL_TRIANGLES, size_array[pos_objects], bunny_size);
		}
		if (show_mode == 'b') {
			for (int i = 0; i < bunny_size / 3; i++) {
				glDrawArrays(GL_LINE_LOOP, size_array[pos_objects] + 3 * i, 3);
			}
		}
		if (show_mode == 'm') {
			update_N("bunny", size_array[pos_objects]);
			glDrawArrays(GL_TRIANGLES, size_array[pos_objects], bunny_size);
		}


	}
}




void angle_change() {

	camera_z = float(distance_r * (float)sin(angle_theta * pi / 180.0f) * (float)cos(angle_phi * pi / 180.0f));
	camera_x = float(distance_r * (float)sin(angle_theta * pi / 180.0f) * (float)sin(angle_phi * pi / 180.0f));

	camera_y = float(distance_r * (float)cos(angle_theta * pi / 180.0f));

	//std::cout<<"camera: "<<camera_x<<" "<<camera_y<<" "<<camera_z<<std::endl;
}
Eigen::Matrix4f InitCameraTransform(const Eigen::Vector3f Target, const Eigen::Vector3f Up) {


	Eigen::Vector3f N1 = -Target;
	N1 = N1.normalized();
	Eigen::Vector3f U1 = Up;
	U1 = U1.cross(Target);
	U1 = U1.normalized();
	Eigen::Vector3f V1 = N1.cross(U1);

	Eigen::Matrix4f m;
	m << U1[0], U1[1], U1[2], 0.0f,
		V1[0], V1[1], V1[2], 0.0f,
		N1[0], N1[1], N1[2], 0.0f,
		0, 0, 0, 1.0f;
	Eigen::Matrix4f n;
	n << 1, 0, 0, -Target[0],
		0, 1, 0, -Target[1],
		0, 0, 1, -Target[2],
		0, 0, 0, 1;
	//std::cout<<"m*n\n"<<m*n<<std::endl;
	return m * n;
}
void rotate(char x, double angle) {
	float sin_angle = sin((pi * angle) / 180.0f);
	float cos_angle = cos((pi * angle) / 180.0f);
	Eigen::Matrix4f rotater;
	if (x == 'x') {
		rotater << cos_angle, -sin_angle, 0, 0,
			sin_angle, cos_angle, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
	}

	if (x == 'y') {
		rotater << cos_angle, 0, -sin_angle, 0,
			0, 1, 0, 0,
			sin_angle, 0, cos_angle, 0,
			0, 0, 0, 1;
	}

	if (x == 'z') {
		rotater << 1, 0, 0, 0,
			0, cos_angle, -sin_angle, 0,
			0, sin_angle, cos_angle, 0,
			0, 0, 0, 1;


	}
	model = model * rotater;
}

void rotate_v(char x) {
	double angle = 10;
	float sin_angle = sin((pi * angle) / 180.0f);
	float cos_angle = cos((pi * angle) / 180.0f);
	Eigen::Matrix4f rotater;


	if (x == 'y') {
		rotater << cos_angle, 0, -sin_angle, 0,
			0, 1, 0, 0,
			sin_angle, 0, cos_angle, 0,
			0, 0, 0, 1;
		int begin_pos = size_array[0];
		int end_pos = size_array[1];
		for (int i = begin_pos; i < end_pos; i++) {
			MatrixXf temp(4, 1);
			temp << V.col(i)[0], V.col(i)[1], V.col(i)[2], 1.0f;
			temp = rotater * temp;
			V.col(i) << temp.col(0)[0], temp.col(0)[1], temp.col(0)[2];
		}

	}

}

void scale(char flag) {
	Eigen::Matrix4f scaler;
	if (flag == '+') {
		scaler << 1.1, 0, 0, 0,
			0, 1.1, 0, 0,
			0, 0, 1.1, 0,
			0, 0, 0, 1;
	}
	if (flag == '-') {
		scaler << 0.9, 0, 0, 0,
			0, 0.9, 0, 0,
			0, 0, 0.9, 0,
			0, 0, 0, 1;
	}


	model = model * scaler;
}

void scale_v(char flag) {
	Eigen::Matrix4f scaler;
	if (flag == '+') {
		scaler << 1.1, 0, 0, 0,
			0, 1.1, 0, 0,
			0, 0, 1.1, 0,
			0, 0, 0, 1;
	}
	if (flag == '-') {
		scaler << 0.9, 0, 0, 0,
			0, 0.9, 0, 0,
			0, 0, 0.9, 0,
			0, 0, 0, 1;
	}

	int begin_pos = size_array[0];
	int end_pos = size_array[1];
	for (int i = begin_pos; i < end_pos; i++) {
		MatrixXf temp(4, 1);
		temp << V.col(i)[0], V.col(i)[1], V.col(i)[2], 1.0f;
		temp = scaler * temp;
		V.col(i) << temp.col(0)[0], temp.col(0)[1], temp.col(0)[2];
	}

}


void move(char flag) {
	Eigen::Matrix4f mover;
	if (flag == 'd') {
		mover << 1, 0, 0, 2,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
	}
	if (flag == 'a') {
		mover << 1, 0, 0, -0.2,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
	}

	if (flag == 'w') {
		mover << 1, 0, 0, 0,
			0, 1, 0, 2,
			0, 0, 1, 0,
			0, 0, 0, 1;
	}

	if (flag == 's') {
		mover << 1, 0, 0, 0,
			0, 1, 0, -2,
			0, 0, 1, 0,
			0, 0, 0, 1;
	}
	model = model * mover;
}

void move_v(char flag) {
	Eigen::Matrix4f mover;
	if (flag == 'd') {
		mover << 1, 0, 0, 2,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
	}
	if (flag == 'a') {
		mover << 1, 0, 0, -0.2,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
	}

	if (flag == 'w') {
		mover << 1, 0, 0, 0,
			0, 1, 0, -0.5,
			0, 0, 1, 0,
			0, 0, 0, 1;
	}

	if (flag == 's') {
		mover << 1, 0, 0, 0,
			0, 1, 0, 0.5,
			0, 0, 1, 0,
			0, 0, 0, 1;
	}

	int begin_pos = size_array[0];
	int end_pos = size_array[1];
	for (int i = begin_pos; i < end_pos; i++) {
		MatrixXf temp(4, 1);
		temp << V.col(i)[0], V.col(i)[1], V.col(i)[2], 1.0f;
		temp = mover * temp;
		V.col(i) << temp.col(0)[0], temp.col(0)[1], temp.col(0)[2];
	}


}

void delete_object() {

	int begin_pos = size_array[0];
	int end_pos = size_array[1];
	for (int i = begin_pos; i < end_pos; i++) {
		V.col(i) << 0,0,0;
	}

}

void generate_phong(Eigen::MatrixXd& V_off, Eigen::MatrixXi& F_off, std::string filename) {
	//std::cout<<"generating phong: "<<filename<<std::endl;
//    std::cout<<"V:\n"<<V_off<<std::endl;
//    std::cout<<"F:\n"<<F_off<<std::endl;
	phong phong_temp;
	for (int i = 0; i < F_off.rows(); i++) {
		int point1_id = F_off.row(i)[0];
		int point2_id = F_off.row(i)[1];
		int point3_id = F_off.row(i)[2];
		//        std::cout<<"point 1 id: "<<point1_id<<" point2 id: "<<point2_id<<" point3 id: "<<point3_id<<std::endl;
		Eigen::Vector3d point1 = V_off.row(point1_id);
		Eigen::Vector3d point2 = V_off.row(point2_id);
		Eigen::Vector3d point3 = V_off.row(point3_id);
		double area = triangle_area(point1, point2, point3);
		phong_temp.area[i] = area;
		Eigen::Vector3d normal = triangle_normal(point1, point2, point3);
		phong_temp.v_normal[point1_id].push_back(std::make_pair(normal, area));
		phong_temp.v_normal[point2_id].push_back(std::make_pair(normal, area));
		phong_temp.v_normal[point3_id].push_back(std::make_pair(normal, area));

		phong_temp.F_off = F_off;
		//        std::cout<<"point 1: "<<point1<<" point2: "<<point2<<" point3: "<<point3<<std::endl;
		//        std::cout<<"area: "<<area<<std::endl;
		//        std::cout<<"normal:\n "<<normal<<std::endl;

	}
	if (filename == "cube") {
		Sphong[0] = phong_temp;
		calculate_average("cube");
	}
	if (filename == "bumpy") {
		Sphong[1] = phong_temp;
		calculate_average("bumpy");
	}
	if (filename == "bunny") {
		Sphong[2] = phong_temp;
		calculate_average("bunny");
	}
}
double triangle_area(Eigen::Vector3d point1, Eigen::Vector3d point2, Eigen::Vector3d point3) {

	Eigen::Vector3d p12 = point1 - point2;
	Eigen::Vector3d p23 = point2 - point3;
	Eigen::Vector3d cross_product = p12.cross(p23);
	double area = sqrt(cross_product.dot(cross_product)) * 0.5;
	//    std::cout<<"cross: "<<sqrt(p12.cross(p23)).dot((p12.cross(p23))))/2<<std::endl;


	return area;
}
Eigen::Vector3d triangle_normal(Eigen::Vector3d point1, Eigen::Vector3d point2, Eigen::Vector3d point3) {
	Eigen::Vector3d p12 = point2 - point1;
	Eigen::Vector3d p13 = point3 - point1;
	//    double nx=p12[1]*p13[2]-p12[2]*p13[1];
	//    double ny=p12[2]*p13[0]-p12[0]*p13[2];
	//    double nz=p12[0]*p13[1]-p12[1]*p13[0];
	Eigen::Vector3d normal = p12.cross(p13);
	if (normal.dot(p12) < 0)
		normal = -normal;
	//std::cout<<"n: "<<nx<<" "<<ny<<" "<<nz<<std::endl;
	//std::cout<<"p12: "<<p12<<" p13: "<<p13<<std::endl;
	//std::cout<<"normal: "<<normal<<std::endl;
	return normal.transpose().normalized();

}
void update_N(std::string filename, int start_pos) {
	//std::cout<<"start position: "<<start_pos<<std::endl;

	if (filename == "cube") {
		for (int row = 0; row < Sphong[0].F_off.rows(); row++) {
			for (int col = 0; col < Sphong[0].F_off.cols(); col++) {
				int point_id = Sphong[0].F_off.row(row)[col];
				Eigen::Vector3d normal_ave = Sphong[0].V_normal_average[point_id];
				//                 std::cout<<"point id: "<<point_id<<" ave normal: "<<normal_ave<<std::endl;
				//                 std::cout<<"point pos: "<<row*3+col<<std::endl;
				N.col(start_pos + row * 3 + col) << normal_ave[0], normal_ave[1], normal_ave[2];

			}
		}

		VBO_normal.update(N);
	}
	if (filename == "bumpy") {
		for (int row = 0; row < Sphong[1].F_off.rows(); row++) {
			for (int col = 0; col < Sphong[1].F_off.cols(); col++) {
				int point_id = Sphong[1].F_off.row(row)[col];
				Eigen::Vector3d normal_ave = Sphong[1].V_normal_average[point_id];
				//                 std::cout<<"point id: "<<point_id<<" ave normal: "<<normal_ave<<std::endl;
				//                 std::cout<<"point pos: "<<row*3+col<<std::endl;
				N.col(start_pos + row * 3 + col) << normal_ave[0], normal_ave[1], normal_ave[2];

			}
		}

		VBO_normal.update(N);
	}

	if (filename == "bunny") {
		for (int row = 0; row < Sphong[2].F_off.rows(); row++) {
			for (int col = 0; col < Sphong[2].F_off.cols(); col++) {
				int point_id = Sphong[2].F_off.row(row)[col];
				Eigen::Vector3d normal_ave = Sphong[2].V_normal_average[point_id];
				N.col(start_pos + row * 3 + col) << normal_ave[0], normal_ave[1], normal_ave[2];

			}
		}

		VBO_normal.update(N);
	}


}
void calculate_average(std::string filename) {

	if (filename == "cube") {
		for (int i = 0; i < 8; i++) {
			double total_area = 0;
			Eigen::Vector3d total_normal(0, 0, 0);
			for (int j = 0; j < Sphong[0].v_normal[i].size(); j++) {
				total_area += Sphong[0].v_normal[i][j].second;
				total_normal = total_normal + (Sphong[0].v_normal[i][j].second * Sphong[0].v_normal[i][j].first);
			}
			Sphong[0].V_normal_average[i] = (total_normal / total_area).normalized();
		}
	}

	if (filename == "bumpy") {
		for (int i = 0; i < 502; i++) {
			double total_area = 0;
			Eigen::Vector3d total_normal(0, 0, 0);
			for (int j = 0; j < Sphong[1].v_normal[i].size(); j++) {
				total_area += Sphong[1].v_normal[i][j].second;
				total_normal = total_normal + (Sphong[1].v_normal[i][j].second * Sphong[1].v_normal[i][j].first);
			}
			Sphong[1].V_normal_average[i] = (total_normal / total_area).normalized();
		}
	}

	if (filename == "bunny") {
		for (int i = 0; i < 502; i++) {
			double total_area = 0;
			Eigen::Vector3d total_normal(0, 0, 0);
			for (int j = 0; j < Sphong[2].v_normal[i].size(); j++) {
				total_area += Sphong[2].v_normal[i][j].second;
				total_normal = total_normal + (Sphong[2].v_normal[i][j].second * Sphong[2].v_normal[i][j].first);
			}
			Sphong[2].V_normal_average[i] = (total_normal / total_area).normalized();
		}
	}

}




void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {

	/***********General operation**********/
	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);
	int width, height;
	glfwGetWindowSize(window, &width, &height);
	double xworld = ((xpos / double(width)) * 2) - 1;
	double yworld = (((height - 1 - ypos) / double(height)) * 2) - 1;
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		//std::cout << "mouse clicking: " <<std::endl;
		std::cout << "V:\n" << V << std::endl;
		std::cout << "V_changed:\n" << V_changed << std::endl;
		mouse_x = xworld;
		mouse_y = yworld;
		std::cout << "mouse: " << mouse_x << "mouse: " << mouse_y << std::endl;
		select_flag = true;
		object_pos = get_object_pos();
		std::cout << "object pos: " << object_pos << std::endl;
	}

	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
		select_flag = false;
	}



	VBO.update(V);
}
int get_object_pos() {

	//    std::cout<<"V:\n"<<V<<std::endl;
	//    std::cout<<"V_changed:\n"<<V_changed<<std::endl;
	Eigen::Vector3d point_plane(camera_x, camera_y, camera_z - near_z);
	Eigen::Vector3d plane_normal(-camera_x, -camera_y, -camera_z);
	plane_normal = plane_normal.normalized();


	for (int i = 0; i < all_object; i++) {
		int start_position = size_array[i];
		int end_position = size_array[i + 1];
		if (object_on_plane(start_position, end_position, point_plane, plane_normal))
			return i;
	}

	return -1;
}
bool object_on_plane(int begin, int end, Vector3d point, Vector3d normal) {
	for (int i = 0; i < (end - begin) / 3; i++) {

		Vector3d p1(V_changed.col(begin + 3 * i)[0], V_changed.col(begin + 3 * i)[1], V_changed.col(begin + 3 * i)[2]);
		Vector3d p2(V_changed.col(begin + 3 * i + 1)[0], V_changed.col(begin + 3 * i + 1)[1], V_changed.col(begin + 3 * i + 1)[2]);
		Vector3d p3(V_changed.col(begin + 3 * i + 2)[0], V_changed.col(begin + 3 * i + 2)[1], V_changed.col(begin + 3 * i + 2)[2]);

		Vector2d projected_p1;
		Vector2d projected_p2;
		Vector2d projected_p3;

		if (view_mode == 'o') {
			projected_p1 = point_projection_o(p1, point, normal.normalized());
			projected_p2 = point_projection_o(p2, point, normal.normalized());
			projected_p3 = point_projection_o(p3, point, normal.normalized());
		}
		if (view_mode == 'p') {
			projected_p1 = point_projection_p(p1, point, normal.normalized());
			projected_p2 = point_projection_p(p2, point, normal.normalized());
			projected_p3 = point_projection_p(p3, point, normal.normalized());
		}
		//         std::cout<<"p1: "<<projected_p1<<std::endl;
		//         std::cout<<"p2: "<<projected_p2<<std::endl;
		//         std::cout<<"p3: "<<projected_p3<<std::endl;

		if (PointInTriangle(mouse_x, mouse_y, projected_p1[0], projected_p1[1], projected_p2[0], projected_p2[1], projected_p3[0], projected_p3[1]))
			return true;



	}

	return false;
}
Vector2d point_projection_o(Vector3d point, Vector3d point_plane, Vector3d normal) {

	double t = (normal[0] * point_plane[0] + normal[1] * point_plane[1] + normal[2] * point_plane[2] - (normal[0] * point[0] + normal[1] * point[1] + normal[2] * point[2])) / (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
	double new_x = point[0] + normal[0] * t;
	double new_y = point[1] + normal[1] * t;

	Vector2d result(new_x, new_y);

	return result;


}

Vector2d point_projection_p(Vector3d point, Vector3d point_plane, Vector3d normal) {

	Vector3d view_point(camera_x, camera_y, camera_z);
	Vector3d l = (point - view_point);
	//std::cout<<"point: "<<point<<std::endl;
	/*std::cout << "l: \n" << l << std::endl;
	std::cout << "upper: " << (((point_plane - view_point)).dot(normal)) << std::endl;
	std::cout << "lower: " << (l.dot(normal)) << std::endl;*/
	double d = (((point_plane - view_point)).dot(normal)) / (l.dot(normal));
	//
	//std::cout << "d: " << d << std::endl;;
	Vector3d p = view_point + d * l;

	//std::cout << "projected point:\n " << p << std::endl;
	Vector2d result(p[0], p[1]);

	return result;
}


void update_v_changed(std::string filename, int start_point, Matrix4f model, Matrix4f projecttion, Matrix4f view) {

	if (filename == "cube") {
		for (int i = 0; i < cube_size; i++) {
			MatrixXf point_temp;
			point_temp.resize(4, 1);
			point_temp << V.col(start_point + i)[0], V.col(start_point + i)[1], V.col(start_point + i)[2], 1.0f;
			point_temp = model * point_temp;
			Vector3f fragPos(point_temp.col(0)[0], point_temp.col(0)[1], point_temp.col(0)[2]);
			//
			MatrixXf point_temp2;
			point_temp2.resize(4, 1);
			point_temp2 << fragPos[0], fragPos[1], fragPos[2], 1.0f;
			Vector4f changed_point = (projecttion * view * point_temp2).col(0).transpose();
			V_changed.col(start_point + i) << changed_point[0], changed_point[1], changed_point[2];
		}
	}


	    if(filename=="bumpy"){
			for (int i = 0; i < bumpy_size; i++) {
				MatrixXf point_temp;
				point_temp.resize(4, 1);
				point_temp << V.col(start_point + i)[0], V.col(start_point + i)[1], V.col(start_point + i)[2], 1.0f;
				point_temp = model * point_temp;
				//            std::cout<<"model: "<<model<<std::endl;
				//            std::cout<<"point temp: "<<point_temp<<std::endl;
						   //Vector3f temp1=(model*point_temp).col(0).transpose();
				Vector3f fragPos(point_temp.col(0)[0], point_temp.col(0)[1], point_temp.col(0)[2]);
				//
				MatrixXf point_temp2;
				point_temp2.resize(4, 1);
				point_temp2 << fragPos[0], fragPos[1], fragPos[2], 1.0f;
				//            std::cout<<"view: "<<view<<std::endl;
							//std::cout<<"projection: "<<projection<<std::endl;
				Vector4f changed_point = (projecttion * view * point_temp2).col(0).transpose();
				//std::cout<<"point: "<<changed_point<<std::endl;
				V_changed.col(start_point + i) << changed_point[0], changed_point[1], changed_point[2];
			}
	    }
	
	    if(filename=="bunny"){
			for (int i = 0; i < bunny_size; i++) {
				MatrixXf point_temp;
				point_temp.resize(4, 1);
				point_temp << V.col(start_point + i)[0], V.col(start_point + i)[1], V.col(start_point + i)[2], 1.0f;
				point_temp = model * point_temp;
				//            std::cout<<"model: "<<model<<std::endl;
				//            std::cout<<"point temp: "<<point_temp<<std::endl;
						   //Vector3f temp1=(model*point_temp).col(0).transpose();
				Vector3f fragPos(point_temp.col(0)[0], point_temp.col(0)[1], point_temp.col(0)[2]);
				//
				MatrixXf point_temp2;
				point_temp2.resize(4, 1);
				point_temp2 << fragPos[0], fragPos[1], fragPos[2], 1.0f;
				//            std::cout<<"view: "<<view<<std::endl;
							//std::cout<<"projection: "<<projection<<std::endl;
				Vector4f changed_point = (projecttion * view * point_temp2).col(0).transpose();
				//std::cout<<"point: "<<changed_point<<std::endl;
				V_changed.col(start_point + i) << changed_point[0], changed_point[1], changed_point[2];
			}
	    }
	//
}


float sign(double p1_x, double p1_y, double p2_x, double p2_y, double p3_x, double p3_y) {
	return (p1_x - p3_x) * (p2_y - p3_y) - (p2_x - p3_x) * (p1_y - p3_y);
}
bool PointInTriangle(double pt_x, double pt_y, double v1_x, double v1_y, double v2_x, double v2_y, double v3_x, double v3_y) {
	float d1, d2, d3;
	bool has_neg, has_pos;

	d1 = sign(pt_x, pt_y, v1_x, v1_y, v2_x, v2_y);
	d2 = sign(pt_x, pt_y, v2_x, v2_y, v3_x, v3_y);
	d3 = sign(pt_x, pt_y, v3_x, v3_y, v1_x, v1_y);

	has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
	has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

	return !(has_neg && has_pos);
}



int main(void) {

	GLFWwindow* window;

	// Initialize the library
	if (!glfwInit())
		return -1;

	// Activate supersampling
	glfwWindowHint(GLFW_SAMPLES, 8);

	// Ensure that we get at least a 3.2 context
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);

#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

	// Create a windowed mode window and its OpenGL context
	window = glfwCreateWindow(640, 640, "Hello World", NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		return -1;
	}

	// Make the window's context current
	glfwMakeContextCurrent(window);

#ifndef __APPLE__
	glewExperimental = true;
	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		/* Problem: glewInit failed, something is seriously wrong. */
		fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
	}
	glGetError(); // pull and savely ignonre unhandled errors like GL_INVALID_ENUM
	fprintf(stdout, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));
#endif


	int major, minor, rev;
	major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
	minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
	rev = glfwGetWindowAttrib(window, GLFW_CONTEXT_REVISION);
	printf("OpenGL version recieved: %d.%d.%d\n", major, minor, rev);
	printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION));
	printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

	VertexArrayObject VAO;
	VAO.init();
	VAO.bind();

	VBO.init();
	V.resize(3, 0);
	VBO.update(V);

	VBO_normal.init();
	N.resize(3, 0);
	VBO_normal.update(N);


	Program program;
	const GLchar* vertex_shader =
		"#version 150 core\n"
		"in vec3 position;"
		"in vec3 aNormal;"
		"out vec3 FragPos;"
		"out vec3 Normal;"

		"uniform mat4 model;"
		"uniform mat4 view;"
		"uniform mat4 projection;"
		"void main()"
		"{"
		"FragPos = vec3(model * vec4(position, 1.0f));"
		"Normal = mat3(transpose(inverse(model))) * aNormal;"
		"gl_Position = projection * view * vec4(FragPos, 1.0);"
		"}";

	const GLchar* fragment_shader =
		"#version 330 core\n"
		"out vec4 outColor;"
		"in vec3 Normal;"
		"in vec3 FragPos;"
		"uniform vec3 lightPos;"
		"uniform vec3 lightColor;"
		"uniform vec3 objectColor;"
		"void main()"
		"{"
		"float ambientStrength = 0.5;"
		"vec3 ambient = ambientStrength * lightColor;"
		"vec3 norm = normalize(Normal);"
		"vec3 lightDir = normalize(lightPos - FragPos);"
		"float diff = max(dot(norm, lightDir), 0.0);"
		"vec3 diffuse = diff * lightColor;"
		"vec3 result = (ambient + diffuse) * objectColor;"
		"outColor = vec4(result, 1.0);"
		"}";
	program.init(vertex_shader, fragment_shader, "outColor");
	program.bind();
	program.bindVertexAttribArray("position", VBO);
	program.bindVertexAttribArray("aNormal", VBO_normal);

	init_matrix(window);
	angle_change();
	//    Eigen::Vector3f Target(camera_x,camera_y,camera_z);
	//    Eigen::Vector3f Up(0,1,0);
	//    Eigen::Matrix4f temp=InitCameraTransform(Target, Up);
	//    std::cout<<"temp: "<<temp<<std::endl;
	glUniform3f(program.uniform("objectColor"), 0.6196f, 0.58f, 0.917647f);
	glUniform3f(program.uniform("lightColor"), 1.0f, 1.0f, 1.0f);
	glUniform3f(program.uniform("lightPos"), 0.0f, 2.0f, -5.0f);

	glUniformMatrix4fv(program.uniform("model"), 1, GL_FALSE, model.data());
	glUniformMatrix4fv(program.uniform("view"), 1, GL_FALSE, view.data());
	glUniformMatrix4fv(program.uniform("projection"), 1, GL_FALSE, projection.data());


	glfwSetKeyCallback(window, key_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);


	while (!glfwWindowShouldClose(window)) {

		VAO.bind();
		program.bind();

		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LESS);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);


		draw_object(program);


		// Swap front and back buffers
		glfwSwapBuffers(window);
		// Poll for and process events
		glfwPollEvents();
	}

	// Deallocate opengl memory
	program.free();
	VAO.free();
	VBO.free();

	// Deallocate glfw internals
	glfwTerminate();
	return 0;


}
