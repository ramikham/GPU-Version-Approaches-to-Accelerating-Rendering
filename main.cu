#include <iostream>
#include <time.h>
#include <float.h>
#include <curand_kernel.h>
#include <fstream>
#include <sstream>
#include "Vec3D.h"
#include "Ray.h"
#include "Sphere.h"
#include "Primitives_List.h"
#include "camera.h"
#include "Material.h"
#include "XY_Rectangle.h"
#include "XZ_Rectangle.h"
#include "YZ_Rectangle.h"
#include "Box.h"
#include "Triangle.h"
#include "BVH.h"
#include "AABB.h"
#include "Rotate_Y.h"

// Constants
// -----------------------------------------------------------------------
const float epsilon = std::numeric_limits<float>::epsilon();

// Mesh Loading Functions
// -----------------------------------------------------------------------
void get_num_vertices_and_faces(const std::string& file_name, int& num_vertices, int& num_faces) {
    // Returns the number of vertices and faces (triangles) a loaded mesh has

    if (num_vertices != 0 && num_faces)
        printf("THE NUMBER OF VERTICES/FACES SHOULD BE = 0 INITIALLY\n");

    std::ifstream obj_file(file_name);

    // Check if file is open
    if (!obj_file.is_open()) {
        std::cerr << "ERROR: UNABLE TO OPEN OBJ FILE " << file_name << std::endl;
        return;
    }

    // Read the file, line-by-line
    std::string line;
    while (std::getline(obj_file, line)) {
        std::istringstream iss(line);
        std::string token;
        iss >> token;

        if (token == "v") {
            num_vertices++;
        }
        if (token == "f") {
            num_faces++;
        }
    }
    obj_file.close();
}

// Loads the OBJ model from file
// -----------------------------------------------------------------------
void load_model(const std::string& file_name, Vec3D* vertices, Vec3D *faces, Vec3D& displacement,
                float& scale_factor, float& X_angle_of_rotation, float& Y_angle_of_rotation, float& Z_angle_of_rotation,
                int num_vertices=0, int num_faces=0) {

    std::ifstream obj_file(file_name);

    // Check if file is open
    if (!obj_file.is_open()) {
        std::cerr << "ERROR: UNABLE TO OPEN OBJ FILE " << file_name << std::endl;
        return;
    }

    // Read the file, line-by-line
    std::string line;
    while (std::getline(obj_file, line)) {
        std::istringstream iss(line);
        std::string token;
        iss >> token;

        if (token == "v") {
            // Vertex
            float x, y, z;
            iss >> x >> y >> z;

            // Apply scaling
            // -----------------------------------------------------------------------
            x *= scale_factor;
            y *= scale_factor;
            z *= scale_factor;

            // Apply rotation
            // -----------------------------------------------------------------------
            // Z-axis rotation
            float theta_Z = Z_angle_of_rotation;
            float tempX = x;
            float tempY = y;

            x = std::cos(theta_Z) * tempX - std::sin(theta_Z) * tempY;
            y = std::sin(theta_Z) * tempX + std::cos(theta_Z) * tempY;

            // Y-axis rotation
            float theta_Y = Y_angle_of_rotation;
            tempX = x;
            float tempZ = z;

            x = std::cos(theta_Y) * tempX + std::sin(theta_Y) * tempZ;
            z = -std::sin(theta_Y) * tempX + std::cos(theta_Y) * tempZ;

            // X-axis rotation
            float theta_X = X_angle_of_rotation;
            tempY = y;
            tempZ = z;

            y = std::cos(theta_X) * tempY - std::sin(theta_X) * tempZ;
            z = std::sin(theta_X) * tempX + std::cos(theta_X) * tempZ;
            // Apply Translation
            // -------------------------------------------------------------------------------
            x += displacement.x();
            y += displacement.y();
            z += displacement.z();

            vertices[num_vertices] = Vec3D(x,y,z);
            num_vertices++;
        } else if (token == "f") {
            // Face
            int v1, v2, v3;
            iss >> v1 >> v2 >> v3;      // DANGER! Ensure obj file follows this for faces

            // Indices in OBJ files start from 1. In C++ they start from 0.
            v1--; v2--; v3--;

            // Pad vertices by epsilon
            Vec3D padded_v1 = vertices[v1] + Vec3D(epsilon, epsilon, epsilon);
            Vec3D padded_v2 = vertices[v2] + Vec3D(epsilon, epsilon, epsilon);
            Vec3D padded_v3 = vertices[v3] + Vec3D(epsilon, epsilon, epsilon);

            //   std::cout << padded_v1 << " " << vertices[v2] << " " << vertices[v3] << " " << std::endl;

            // Create the triangle using the vertices
            faces[num_faces] = padded_v1;
            num_faces++;
            faces[num_faces] = padded_v2;
            num_faces++;
            faces[num_faces] = padded_v3;
            num_faces++;
        }
    }
    obj_file.close();
}

// Copied from helper_cuda.h in CUDA examples
// -------------------------------------------------------------------------------
#define checkCudaErrors(val) check_cuda( (val), #val, __FILE__, __LINE__ )
void check_cuda(cudaError_t result, char const *const func, const char *const file, int const line) {
    if (result) {
        std::cerr << "CUDA error = " << static_cast<unsigned int>(result) << " at " <<
                  file << ":" << line << " '" << func << "' \n";
        cudaDeviceReset();
        exit(99);
    }
}

__device__ Vec3D radiance(const Ray& r, Primitive **world, curandState *local_rand_state) {
    Ray cur_ray = r;
    Vec3D cur_attenuation = Vec3D(1.0, 1.0, 1.0);
    for(int i = 0; i < 5; i++) {
        Intersection_Information rec;
        if ((*world)->intersection(cur_ray, 0.001f, FLT_MAX, rec)) {
            Ray scattered;
            Vec3D attenuation;
            float pdf;
            PDF* surface_pdf_ptr = nullptr;

            if(rec.mat_ptr->evaluate(cur_ray, rec, attenuation, scattered, local_rand_state, pdf, surface_pdf_ptr)) {
                cur_attenuation *= attenuation;
                cur_ray = scattered;
            }
            else {
                return Vec3D(0.0, 0.0, 0.0);
            }
        }
        else {
            Vec3D unit_direction = unit_vector(cur_ray.get_ray_direction());
            float t = 0.5f*(unit_direction.y() + 1.0f);
            Vec3D c = (1.0f - t) * Vec3D(1.0, 1.0, 1.0) + t * Vec3D(0.5, 0.7, 1.0);
            return cur_attenuation * c;
        }
    }
    return Vec3D(0.0, 0.0, 0.0); // exceeded recursion
}

__device__ Vec3D radiance_background(const Ray& r, Primitive **world, Vec3D background_color, curandState *local_rand_state) {
    Ray cur_ray = r;
    Vec3D curr_attenuation = Vec3D(1.0, 1.0, 1.0);
    Vec3D light_accum = Vec3D(0.0, 0.0, 0.0);

    for (int i = 0; i < 10; i++) {
        Intersection_Information rec;
        if ((*world)->intersection(cur_ray, 0.001f, FLT_MAX, rec)) {
            Ray scattered_ray;
            Vec3D attenuation;
            float pdf;
            PDF* surface_pdf_ptr = nullptr;

            Vec3D color_from_emission = rec.mat_ptr->emitted(rec.p, rec);

            if (rec.mat_ptr->evaluate(cur_ray, rec, attenuation, scattered_ray, local_rand_state, pdf, surface_pdf_ptr)) {
                light_accum += color_from_emission * curr_attenuation / pdf;
                curr_attenuation *= attenuation;
                cur_ray = scattered_ray;
            } else {
                return light_accum/pdf + color_from_emission * curr_attenuation;
            }
        } else {
            return light_accum + background_color * curr_attenuation;
        }
    }
    return Vec3D(0.0, 0.0, 0.0);
}

__device__ Vec3D radiance_mixture(const Ray& r, Primitive **world, Vec3D background_color, curandState *local_rand_state) {
    Ray cur_ray = r;
    Vec3D curr_attenuation = Vec3D(1.0, 1.0, 1.0);
    Vec3D light_accum = Vec3D(0.0, 0.0, 0.0);

    for (int i = 0; i < 10; i++) {
        Intersection_Information rec;
        if ((*world)->intersection(cur_ray, 0.001f, FLT_MAX, rec)) {
            Ray scattered_ray;
            Vec3D attenuation;
            float pdf;
            PDF* surface_pdf_ptr = nullptr;

            Vec3D color_from_emission = rec.mat_ptr->emitted(rec.p, rec);

            if (rec.mat_ptr->evaluate(cur_ray, rec, attenuation, scattered_ray, local_rand_state, pdf, surface_pdf_ptr)) {
                light_accum += color_from_emission * curr_attenuation / pdf;
                curr_attenuation *= attenuation;
                cur_ray = scattered_ray;
            } else {
                return light_accum/pdf + color_from_emission * curr_attenuation;
            }
        } else {
            return light_accum + background_color * curr_attenuation;
        }
    }
    return Vec3D(0.0, 0.0, 0.0);
}

// radiance() function with background color enabled. Set to (0.0,0.0,0.0) to get a dark background,
// and experiment with rendering lights.
// Color(0.70, 0.80, 1.00) is sky blue
// (0,0,0) is dark
// (0.04, 0.04, 0.08) sky dark
__device__ Vec3D radiance_background_recursive(curandState *local_rand_state, const Ray& r, Primitive **world, int depth= 10, Vec3D background=Vec3D(0.7, 0.8, 1.00)){
    Intersection_Information rec;
    if (depth <= 0)
        return Vec3D(0,0,0);

    if ((*world)->intersection(r, 0.001, FLT_MAX, rec))
        // Background color when there is no intersection
        return background;

    Ray scattered_ray;
    Vec3D surface_color;
    float pdf;
    PDF* surface_pdf_ptr = nullptr;

    Vec3D color_from_emission = rec.mat_ptr->emitted(rec.p, rec);

    if (!rec.mat_ptr->evaluate(r, rec, surface_color, scattered_ray, local_rand_state, pdf, surface_pdf_ptr))
        return color_from_emission;

    return color_from_emission + rec.mat_ptr->BRDF(r, rec, scattered_ray, surface_color) *
                                 radiance_background_recursive(local_rand_state, scattered_ray, world, depth-1, background) / pdf;
}
// Initialize the state of a random number generator. To be executed by only on thread
// (let it be the first).
// -------------------------------------------------------------------------------
__global__ void rand_init(curandState *rand_state) {
    if (threadIdx.x == 0 && blockIdx.x == 0) {
        curand_init(3019, 0, 0, rand_state);
    }
}

// Render loop
// -------------------------------------------------------------------------------
__global__ void render(Vec3D *pixel_colros, int image_width, int image_height, int samples_per_pixel, camera **cam, Primitive **world, curandState *rand_state) {
    int i = threadIdx.x + blockIdx.x * blockDim.x;
    int j = threadIdx.y + blockIdx.y * blockDim.y;

    if((i >= image_width) || (j >= image_height))
        return;

    int pixel_index = j * image_width + i;

    // Unique seed for each pixel
    curand_init(3019 + pixel_index, 0, 0, &rand_state[pixel_index]);

    curandState local_rand_state = rand_state[pixel_index];
    Vec3D pixel_color(0, 0, 0);
    for(int s=0; s < samples_per_pixel; s++) {
        float u = float(i + curand_uniform(&local_rand_state)) / float(image_width);
        float v = float(j + curand_uniform(&local_rand_state)) / float(image_height);
        Ray r = (*cam)->get_ray(u, v, &local_rand_state);
         pixel_color += radiance(r, world, &local_rand_state);                                          // working
        // pixel_color += radiance_background(r, world, Vec3D(0.0, 0.0, 0.0), &local_rand_state);       // working
        // pixel_color += radiance_background_recursive(&local_rand_state, r, world);
    }
    rand_state[pixel_index] = local_rand_state;

    pixel_color /= float(samples_per_pixel);
    pixel_color[0] = sqrt(pixel_color[0]);
    pixel_color[1] = sqrt(pixel_color[1]);
    pixel_color[2] = sqrt(pixel_color[2]);

    pixel_colros[pixel_index] = pixel_color;
}

#define RND (curand_uniform(&local_rand_state))

__global__ void create_one_weekend_world(Primitive **d_list, Primitive **d_world, camera **d_camera, int nx, int ny, curandState *rand_state) {
    if (threadIdx.x == 0 && blockIdx.x == 0) {
        curandState local_rand_state = *rand_state;

        // Materials
        auto ground_material = new lambertian(Vec3D(0.5,0.5,0.5));
        auto material_1 = new metal(Vec3D(0.81, 0.85, 0.88), 0.0);
        auto material_2 = new lambertian(Vec3D(0.4, 0.2, 0.1));
        auto material_3 = new metal(Vec3D(0.7, 0.6, 0.5), 0.0);


        d_list[0] = new Sphere(Vec3D(0, -1000.0, -1), 1000, ground_material);
        int i = 1;
        for(int a = -11; a < 11; a++) {
            for(int b = -11; b < 11; b++) {
                float choose_mat = RND;
                Vec3D center(a + RND, 0.2, b + RND);
                if(choose_mat < 0.8f) {
                    d_list[i++] = new Sphere(center, 0.2,
                                             new lambertian(Vec3D(RND * RND, RND * RND, RND * RND)));
                }
                else if(choose_mat < 0.95f) {
                    d_list[i++] = new Sphere(center, 0.2,
                                             new metal(Vec3D(0.5f * (1.0f + RND), 0.5f * (1.0f + RND), 0.5f * (1.0f + RND)), 0.5f * RND));
                }
                else {
                    d_list[i++] = new Sphere(center, 0.2, new dielectric(1.5));
                }
            }
        }

        d_list[i++] = new Sphere(Vec3D(0, 1, 0), 1.0, material_1);
        d_list[i++] = new Sphere(Vec3D(-4, 1, 0), 1.0, material_2);
        d_list[i++] = new Sphere(Vec3D(4, 1, 0), 1.0, material_3);
        d_list[i++] = new Sphere(Vec3D(4, 1, 0), 1.0, material_3);

        *rand_state = local_rand_state;
        int j = i;

        auto bla_world = new Primitives_List(d_list, j);
        auto bla_bvh = new BVH(bla_world->list, i, 0.0, 0.0, &local_rand_state);
        *d_world = bla_bvh;

       // *d_world  = new Primitives_List(d_list, j);       commented for now - hopefully BVH works

        printf("Scene has: %d primitives (including BVH)\n", j);

        Vec3D lookfrom(13, 2, 3);
        Vec3D lookat(0, 0, 0);
        float dist_to_focus = 10.0; (lookfrom-lookat).length();
        float aperture = 0.1;
        *d_camera   = new camera(lookfrom,
                                 lookat,
                                 Vec3D(0, 1, 0),
                                 30.0,
                                 float(nx)/float(ny),
                                 aperture,
                                 dist_to_focus);
    }
}

__global__ void free_one_weekend_world(Primitive **d_list, Primitive **d_world, camera **d_camera, int num_of_primitives_to_delete) {
    if (threadIdx.x == 0 && blockIdx.x == 0) {
        delete* d_world;
        delete* d_camera;
    }
}


__global__ void create_Cornell_Box_world(Vec3D* faces, int num_faces, Primitive **d_list, Primitive **d_world, camera **d_camera, int nx, int ny, curandState *rand_state) {
    if (threadIdx.x == 0 && blockIdx.x == 0) {
        curandState local_rand_state = *rand_state;
        int i = 0;

        // Materials
        auto red = new lambertian(Vec3D(0.65, 0.05, 0.05));
        auto green = new lambertian(Vec3D(0.12, 0.45, 0.15));
        auto white = new lambertian(Vec3D(0.73, 0.73, 0.73));
        auto light = new Diffuse_Light(Vec3D(15, 15, 15));

        auto mirror = new metal(Vec3D(0.8, 0.85, 0.88), 0.0);

        // Primitives
        d_list[i++] = new YZ_Rectangle(Vec3D(555, 0, 0), Vec3D(555, 555, 555), green);
        d_list[i++] = new YZ_Rectangle(Vec3D(0,0,0), Vec3D(0, 555, 555), red);

        d_list[i++] = new XY_Rectangle(Vec3D(0, 0, 555), Vec3D(555, 555, 555), mirror);

        // d_list[i++] = new XZ_Rectangle(Vec3D(213, 554, 227), Vec3D(343,554,332), light); // was light
        d_list[i++] = new XZ_Rectangle(Vec3D(0, 0, 0), Vec3D(555,0,555), white);
        d_list[i++] = new XZ_Rectangle(Vec3D(0, 555, 0), Vec3D(555,555,555), white);

        // d_list[i++] = new Box(Vec3D(130, 0, 65), Vec3D(295, 165, 230), white);
        // d_list[i++] = new Box(Vec3D(265, 0, 295), Vec3D(430, 330, 460), white);

        for (int x = 0; x < num_faces * 3; x += 3) {
            Vec3D A, B, C;
            A = faces[x];
            B = faces[x + 1];
            C = faces[x + 2];
            d_list[i++] = new Triangle(A, B, C, red);
            // printf("done\n");
        }

        //CORRECT
        int j = i;
        printf("HOST: done BVH FOR BOXES\n");
        *rand_state = local_rand_state;


        auto bla_world = new Primitives_List(d_list, j);
        auto bla_bvh = new BVH(bla_world->list, i, 0.0, 0.0, &local_rand_state);
        *d_world = bla_bvh;
        printf("Scene has: %d primitives (including BVH)\n", j);

       // *d_world = new Primitives_List(d_list, j);            // without bvh - working


        printf("HOST: done BVH FOR TRIANGLES\n");
        // Camera settings
        Vec3D lookfrom(278, 278, -800);
        Vec3D lookat(278, 278, 0);
        Vec3D vup(0,1,0);
        float vfov = 40.0;
        float aspect_ratio = 1.0;
        float aperture = 0.0;
        float dist_to_focus = 1.0;
        *d_camera   = new camera(lookfrom,
                                 lookat,
                                 vup,
                                 vfov,
                                 aspect_ratio,
                                 aperture,
                                 dist_to_focus);
    }
}

__global__ void free_Cornell_Box_world(Primitive **d_list, Primitive **d_world, camera **d_camera, int num_of_primitives_to_delete) {
    if (threadIdx.x == 0 && blockIdx.x == 0) {
        delete* d_world;
        delete* d_camera;
    }
}

void one_weekend_scene() {
    // Scene Information
    // -------------------------------------------------------------------------------
    cudaDeviceSetLimit(cudaLimitStackSize, 65536);
    int image_width = 1200;
    int image_height = 675;
    int samples_per_pixel = 100;
    int threads_x = 128;
    int threads_y = 8;

    std::cerr << "Image Width = " << image_width << "\n";
    std::cerr << "Image Height = " << image_height << "\n";
    std::cerr << "Threads across x and y: " << threads_x << "x" << threads_y << "\n";

    // Allocate frame buffer to hold final rendered image
    // -------------------------------------------------------------------------------
    int num_pixels = image_width * image_height;
    size_t fb_size = num_pixels*sizeof(Vec3D);
    Vec3D *fb;
    checkCudaErrors(cudaMallocManaged((void **)&fb, fb_size));

    // Allocate random state
    // -------------------------------------------------------------------------------
    curandState *d_rand_state;
    checkCudaErrors(cudaMalloc((void **)&d_rand_state, num_pixels*sizeof(curandState)));
    curandState *d_rand_state2;
    checkCudaErrors(cudaMalloc((void **)&d_rand_state2, 1*sizeof(curandState)));

    // Initialize the second random state for scene creation
    // ------------------------------------------------------------------------------
    rand_init<<<1,1>>>(d_rand_state2);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    // Copy the list of primitives from the CPU to the GPU before using it in the kernel
    // -------------------------------------------------------------------------------
    Primitive **d_list;
    int num_primitives = 488+1;
    checkCudaErrors(cudaMalloc((void **)&d_list, num_primitives * sizeof(Primitive *)));

    // Copy the "world" from the CPU to the GPU before using it in the kernel. The
    // "world" will be populated by all objects in the scene.
    // -------------------------------------------------------------------------------
    Primitive **d_world;
    checkCudaErrors(cudaMalloc((void **)&d_world, sizeof(Primitive *)));

    // Copy the camera from the CPU to the GPU before using it in the kernel
    // -------------------------------------------------------------------------------
    camera **d_camera;
    checkCudaErrors(cudaMalloc((void **)&d_camera, sizeof(camera *)));

    // Launch the scene kernel to build the scene (by only one thread)
    // -------------------------------------------------------------------------------
    create_one_weekend_world<<<1,1>>>(d_list, d_world, d_camera, image_width, image_height, d_rand_state2);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    // Launch the rendering kernel
    // -------------------------------------------------------------------------------
    clock_t start, end;
    start = clock();

    // Specify number of blocks and threads per block
    dim3 blocks(image_width / threads_x + 1, image_height / threads_y + 1);
    dim3 threads(threads_x, threads_y);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    // Launch render kernel
    render<<<blocks, threads>>>(fb, image_width, image_height, samples_per_pixel, d_camera, d_world, d_rand_state);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    end = clock();
    double total_time = ((double)(end - start)) / CLOCKS_PER_SEC;
    std::cerr << "Total time = " << total_time << " seconds.\n";


    // Write to the output .ppm file
    // -------------------------------------------------------------------------------
    const std::string& file_name = "One Weekend Scene.ppm";
    std::ofstream ofs(file_name, std::ios_base::out | std::ios_base::binary);

    ofs << "P3\n" << image_width << " " << image_height << "\n255\n";
    for (int j = image_height - 1; j >= 0; j--) {
        for (int i = 0; i < image_width; i++) {
            size_t pixel_index = j * image_width + i;
            int ir = int(255.99*fb[pixel_index].x());
            int ig = int(255.99*fb[pixel_index].y());
            int ib = int(255.99*fb[pixel_index].z());
            ofs << ir << " " << ig << " " << ib << "\n";
        }
    }

    ofs.close();

    // Clean all allocated memory
    // -------------------------------------------------------------------------------
    checkCudaErrors(cudaDeviceSynchronize());
    free_one_weekend_world<<<1,1>>>(d_list, d_world, d_camera, num_primitives);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaFree(d_camera));
    checkCudaErrors(cudaFree(d_world));
    checkCudaErrors(cudaFree(d_list));
    checkCudaErrors(cudaFree(d_rand_state));
    checkCudaErrors(cudaFree(d_rand_state2));
    checkCudaErrors(cudaFree(fb));

    // Reset device
    // -------------------------------------------------------------------------------
    cudaDeviceReset();
}

void teapot_scene() {
    // Scene Information
    // -------------------------------------------------------------------------------
    cudaDeviceSetLimit(cudaLimitStackSize, 65536);
    int image_width = 800;
    int image_height = 800;
    int samples_per_pixel = 100;
    int threads_x = 4;        // num of threads_x
    int threads_y = 4;        // num of threads_y

    std::cerr << "Image Width = " << image_width << "\n";
    std::cerr << "Image Height = " << image_height << "\n";
    std::cerr << "Threads across x and y: " << threads_x << "x" << threads_y << "\n";

    // Allocate frame buffer to hold final rendered image
    // -------------------------------------------------------------------------------
    int num_pixels = image_width * image_height;
    size_t fb_size = num_pixels * sizeof(Vec3D);
    Vec3D *fb;
    checkCudaErrors(cudaMallocManaged((void **)&fb, fb_size));

    // Allocate random state
    // -------------------------------------------------------------------------------
    curandState *d_rand_state_1;
    checkCudaErrors(cudaMalloc((void **)&d_rand_state_1, num_pixels * sizeof(curandState)));
    curandState *d_rand_state_2;
    checkCudaErrors(cudaMalloc((void **)&d_rand_state_2, 1 * sizeof(curandState)));

    // Initialize the second random state for scene creation
    // -------------------------------------------------------------------------------
    rand_init<<<1,1>>>(d_rand_state_2);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    // Initialize the meshes
    // -------------------------------------------------------------------------------
    int num_vertices = 0;
    int num_faces = 0;
    get_num_vertices_and_faces("C:\\Users\\ramik\\OneDrive\\Desktop\\Folders\\Models\\teapot.obj", num_vertices, num_faces);

    auto* vertices = (Vec3D*)calloc(num_vertices, sizeof(Vec3D));
    auto* faces = (Vec3D*)calloc(num_faces * 3, sizeof(Vec3D));

    // Mesh transformation information
    Vec3D displacement(277.5, -0.5, 210.0);
    float scale_factor = 40.0f;
    float X_angle_of_rotation = 0.0f;
    float Y_angle_of_rotation = -59.5f;
    float Z_angle_of_rotation = 0.0f;

    // Load the mesh
    load_model("C:\\Users\\ramik\\OneDrive\\Desktop\\Folders\\Models\\teapot.obj", vertices, faces, displacement, scale_factor, X_angle_of_rotation, Y_angle_of_rotation, Z_angle_of_rotation);

    // Copy the mesh data from the CPU to the GPU before using it in the kernel
    // -------------------------------------------------------------------------------

    // Allocate memory on the device for faces
    Vec3D* d_faces;
    checkCudaErrors(cudaMalloc((void**)&d_faces, num_faces * 3 * sizeof(Vec3D)));

    // Copy faces from the host memory to the device memory
    checkCudaErrors(cudaMemcpy(d_faces, faces, num_faces * 3 * sizeof(Vec3D), cudaMemcpyHostToDevice));

    // Copy the list of primitives from the CPU to the GPU before using it in the kernel
    // -------------------------------------------------------------------------------
    Primitive **d_list;
    int num_primitives = 5 + num_faces + 1;       // number of primitives = 8 (rectangles) + num_faces + 1 (BVH)
    checkCudaErrors(cudaMalloc((void **)&d_list, num_primitives * sizeof(Primitive *)));

    // Copy the "world" from the CPU to the GPU before using it in the kernel. The
    // "world" will be populated by all objects in the scene.
    // -------------------------------------------------------------------------------
    Primitive **d_world;
    checkCudaErrors(cudaMalloc((void **)&d_world, sizeof(Primitive *)));

    // Copy the camera from the CPU to the GPU before using it in the kernel
    // -------------------------------------------------------------------------------
    camera **d_camera;
    checkCudaErrors(cudaMalloc((void **)&d_camera, sizeof(camera *)));

    // Launch the scene kernel to build the scene (by only one thread)
    // -------------------------------------------------------------------------------
    create_Cornell_Box_world<<<1,1>>>(d_faces, num_faces, d_list, d_world, d_camera, image_width, image_height, d_rand_state_2);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    // Launch the rendering kernel
    // -------------------------------------------------------------------------------
    clock_t start, end;
    start = clock();

    // Specify number of blocks and threads per block
    dim3 blocks(image_width / threads_x + 1, image_height / threads_y + 1);                // specify number of blocks
    dim3 threads(threads_x, threads_y);                                                    // specify number of threads per block
    render<<<blocks, threads>>>(fb, image_width, image_height, samples_per_pixel, d_camera, d_world, d_rand_state_1);

    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    end = clock();
    double total_time = ((double)(end - start)) / CLOCKS_PER_SEC;
    std::cerr << "Total time = " << total_time << " seconds.\n";

    // Write to the output .ppm file
    // -------------------------------------------------------------------------------
    const std::string& file_name = "Mirror & Teapot Scene.ppm";
    std::ofstream ofs(file_name, std::ios_base::out | std::ios_base::binary);
    ofs << "P3\n" << image_width << " " << image_height << "\n255\n";
    for (int j = image_height - 1; j >= 0; j--) {
        for (int i = 0; i < image_width; i++) {
            size_t pixel_index = j * image_width + i;
            int ir = int(255.99*fb[pixel_index].x());
            int ig = int(255.99*fb[pixel_index].y());
            int ib = int(255.99*fb[pixel_index].z());
            ofs << ir << " " << ig << " " << ib << "\n";
        }
    }

    ofs.close();

    // Clean all allocated memory
    // -------------------------------------------------------------------------------
    checkCudaErrors(cudaDeviceSynchronize());
    free_Cornell_Box_world<<<1,1>>>(d_list,d_world,d_camera, 8+num_faces);
    checkCudaErrors(cudaFree(d_camera));
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaFree(d_world));
    checkCudaErrors(cudaFree(d_list));
    checkCudaErrors(cudaFree(d_rand_state_1));
    checkCudaErrors(cudaFree(d_rand_state_2));
    checkCudaErrors(cudaFree(fb));

    // Reset Device
    // -------------------------------------------------------------------------------
    cudaDeviceReset();
}

int main() {
    // Some device information
    // -------------------------------------------------------------------------------
    int deviceID = 0;
    cudaDeviceProp deviceProp;

    cudaGetDeviceProperties(&deviceProp, deviceID);

    std::cout << "Maximum Threads Per Block: " << deviceProp.maxThreadsPerBlock << std::endl;
    // Calculate the maximum number of blocks based on the maximum grid dimensions
    int maxBlocksX = deviceProp.maxGridSize[0];
    int maxBlocksY = deviceProp.maxGridSize[1];
    int maxBlocksZ = deviceProp.maxGridSize[2];

    int maxBlocks = maxBlocksX * maxBlocksY * maxBlocksZ;

    std::cout << "Maximum Blocks in X Dimension: " << maxBlocksX << std::endl;
    std::cout << "Maximum Blocks in Y Dimension: " << maxBlocksY << std::endl;
    std::cout << "Maximum Blocks in Z Dimension: " << maxBlocksZ << std::endl;
    std::cout << "Maximum Total Blocks: " << maxBlocks << std::endl;
    printf("---------------------------------------------------------------------------------\n");
    // -------------------------------------------------------------------------------

    /* -------------------------------------Teapot Scene------------------------------------------ */
     // teapot_scene();
    /* ------------------------------------------------------------------------------------------- */

    /* -------------------------------------One Weekend Scene------------------------------------------ */
    one_weekend_scene();
    /* ------------------------------------------------------------------------------------------------ */
}