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
#include "Empty.h"


const float epsilon = std::numeric_limits<float>::epsilon();
//  int num_vertices, Triangle* triangles, int num_triangles, const Material* material, Vec3D& displacement, float& scale_factor, float& angle_of_rotation)
void get_num_vertices_and_faces(const std::string& file_name, int& num_vertices, int& num_faces) {
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

void load_model(const std::string& file_name, Vec3D* vertices, Vec3D *faces, Vec3D& displacement, float& scale_factor, float& angle_of_rotation, int num_vertices=0, int num_faces=0) {
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
            x *= scale_factor;
            y *= scale_factor;
            z *= scale_factor;

            float theta = angle_of_rotation;  // - 59.5
            float tempX = x;
            float tempZ = z;

            x = std::cos(theta) * tempX - std::sin(theta) * tempZ;
            z = std::sin(theta) * tempX + std::cos(theta) * tempZ;

            x += displacement.x();
            y += displacement.y();
            z += displacement.z();


            vertices[num_vertices] = Vec3D(x,y,z);
            //    std::cout << x << " " << y << " " << z << " " << std::endl;
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
/*
void load_model(const std::string& file_name,std::vector<Vec3D>& vertices,
                std::vector<Triangle>& triangles, const std::shared_ptr<Material>& material,
                Vec3D& displacement, float& scale_factor, float& angle_of_rotation){
    std::ifstream obj_file(file_name);

    if (!obj_file.is_open()) {
        std::cerr << "ERROR: UNABLE TO OPEN OBJ FILE " << file_name << std::endl;
        return;
    }

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
            x *= scale_factor;
            y *= scale_factor;
            z *= scale_factor;

            float theta = angle_of_rotation;  // - 59.5
            float tempX = x;
            float tempZ = z;

            x = std::cos(theta) * tempX - std::sin(theta) * tempZ;
            z = std::sin(theta) * tempX + std::cos(theta) * tempZ;

            x += displacement.x();
            y += displacement.y();
            z += displacement.z();

            vertices.emplace_back(x, y, z);
            //    std::cout << x << " " << y << " " << z << " " << std::endl;
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
            triangles.emplace_back(padded_v1, padded_v2, padded_v3, material);
        }
    }
    obj_file.close();
}
 */

// limited version of checkCudaErrors from helper_cuda.h in CUDA examples
#define checkCudaErrors(val) check_cuda( (val), #val, __FILE__, __LINE__ )

void check_cuda(cudaError_t result, char const *const func, const char *const file, int const line) {
    if (result) {
        std::cerr << "CUDA error = " << static_cast<unsigned int>(result) << " at " <<
                  file << ":" << line << " '" << func << "' \n";
        // Make sure we call CUDA Device Reset before exiting
        cudaDeviceReset();
        exit(99);
    }
}

// Matching the C++ code would recurse enough into color() calls that
// it was blowing up the stack, so we have to turn this into a
// limited-depth loop instead.  Later code in the book limits to a max
// depth of 50, so we adapt this a few chapters early on the GPU.
__device__ Vec3D color(const Ray& r, Primitive **world, curandState *local_rand_state) {
    Ray cur_ray = r;
    Vec3D cur_attenuation = Vec3D(1.0, 1.0, 1.0);
    for(int i = 0; i < 10; i++) {
        Intersection_Information rec;
        if ((*world)->intersection(cur_ray, 0.001f, FLT_MAX, rec)) {
            Ray scattered;
            Vec3D attenuation;
            if(rec.mat_ptr->scatter(cur_ray, rec, attenuation, scattered, local_rand_state)) {
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
            Vec3D this_emitted_color = rec.mat_ptr->emitted(rec.p, rec);

            Vec3D color_from_emission = rec.mat_ptr->emitted(rec.p, rec);
            if (rec.mat_ptr->scatter(cur_ray, rec, attenuation, scattered_ray, local_rand_state)) {
                light_accum += this_emitted_color * curr_attenuation;
                curr_attenuation *= attenuation;
                cur_ray = scattered_ray;
            } else {
                return light_accum + this_emitted_color * curr_attenuation;
            }
        } else {
            return light_accum + background_color * curr_attenuation;
        }
    }
    return Vec3D(0.0, 0.0, 0.0);
}

__global__ void rand_init(curandState *rand_state) {
    if (threadIdx.x == 0 && blockIdx.x == 0) {
        curand_init(1984, 0, 0, rand_state);
    }
}

__global__ void render_init(int max_x, int max_y, curandState *rand_state) {
    int i = threadIdx.x + blockIdx.x * blockDim.x;
    int j = threadIdx.y + blockIdx.y * blockDim.y;
    if((i >= max_x) || (j >= max_y)) return;
    int pixel_index = j*max_x + i;
    // Original: Each thread gets same seed, a different sequence number, no offset
    // curand_init(1984, pixel_index, 0, &rand_state[pixel_index]);
    // BUGFIX, see Issue#2: Each thread gets different seed, same sequence for
    // performance improvement of about 2x!
    curand_init(1984+pixel_index, 0, 0, &rand_state[pixel_index]);
}

__global__ void render(Vec3D *fb, int max_x, int max_y, int ns, camera **cam, Primitive **world, curandState *rand_state) {
    int i = threadIdx.x + blockIdx.x * blockDim.x;
    int j = threadIdx.y + blockIdx.y * blockDim.y;
    if((i >= max_x) || (j >= max_y)) return;
    int pixel_index = j*max_x + i;
    curandState local_rand_state = rand_state[pixel_index];
    Vec3D col(0, 0, 0);
    for(int s=0; s < ns; s++) {
        float u = float(i + curand_uniform(&local_rand_state)) / float(max_x);
        float v = float(j + curand_uniform(&local_rand_state)) / float(max_y);
        Ray r = (*cam)->get_ray(u, v, &local_rand_state);
         col += color(r, world, &local_rand_state);
       // col += radiance_background(r, world, Vec3D(0.0,0.0,0.0), &local_rand_state);
    }
    rand_state[pixel_index] = local_rand_state;
    col /= float(ns);
    col[0] = sqrt(col[0]);
    col[1] = sqrt(col[1]);
    col[2] = sqrt(col[2]);
    fb[pixel_index] = col;
}

#define RND (curand_uniform(&local_rand_state))

__global__ void create_world(Primitive **d_list, Primitive **d_world, camera **d_camera, int nx, int ny, curandState *rand_state) {
    if (threadIdx.x == 0 && blockIdx.x == 0) {
        curandState local_rand_state = *rand_state;
        d_list[0] = new Sphere(Vec3D(0, -1000.0, -1), 1000,
                               new lambertian(Vec3D(0.5, 0.5, 0.5)));
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
        d_list[i++] = new Sphere(Vec3D(0, 1, 0), 1.0, new dielectric(1.5));
        d_list[i++] = new Sphere(Vec3D(-4, 1, 0), 1.0, new lambertian(Vec3D(0.4, 0.2, 0.1)));
        d_list[i++] = new Sphere(Vec3D(4, 1, 0), 1.0, new metal(Vec3D(0.7, 0.6, 0.5), 0.0));
        *rand_state = local_rand_state;
        int j = i;
        d_list[j++] = new BVH(d_list, i, 0.0,  1.0 , &local_rand_state);
        *d_world  = new Primitives_List(d_list, 22 * 22 + 1 + 3 + 1);

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

__global__ void create_Cornell_Box_world(Vec3D* faces, int num_faces, Primitive **d_list, Primitive **d_world, camera **d_camera, int nx, int ny, curandState *rand_state) {
    if (threadIdx.x == 0 && blockIdx.x == 0) {
        curandState local_rand_state = *rand_state;
        int i = 0;

        // Materials
        auto red = new lambertian(Vec3D(0.65, 0.05, 0.05));
        auto green = new lambertian(Vec3D(0.12, 0.45, 0.15));
        auto white = new lambertian(Vec3D(0.73, 0.73, 0.73));
        auto light = new Diffuse_Light(Vec3D(15, 15, 15));

        // Primitives
        d_list[i++] = new YZ_Rectangle(Vec3D(555, 0, 0), Vec3D(555, 555, 555), green);
        d_list[i++] = new YZ_Rectangle(Vec3D(0,0,0), Vec3D(0, 555, 555), red);

        d_list[i++] = new XY_Rectangle(Vec3D(0, 0, 555), Vec3D(555, 555, 555), white);

        d_list[i++] = new XZ_Rectangle(Vec3D(213, 554, 227), Vec3D(343,554,332), light); // was light
        d_list[i++] = new XZ_Rectangle(Vec3D(0, 0, 0), Vec3D(555,0,555), white);
        d_list[i++] = new XZ_Rectangle(Vec3D(0, 555, 0), Vec3D(555,555,555), white);

        d_list[i++] = new Box(Vec3D(130, 0, 65), Vec3D(295, 165, 230), white);
        d_list[i++] = new Box(Vec3D(265, 0, 295), Vec3D(430, 330, 460), white);

        //  d_list[i++] = new BVH(new Primitives_List(d_list, i));// check d_list type or maybe use d_world like below
        /* correct routine
        auto bl = new Primitives_List();
        auto bb = new BVH(*bl);
         d_list[i++] = bb;
        */


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
        d_list[j++] = new BVH(d_list, i, 0.0, 0.0 , &local_rand_state);
        *rand_state = local_rand_state;

        *d_world = new Primitives_List(d_list, j);            // without bvh - working
        printf("HOST: done BVH FOR TRIANGLES\n");
        // Camera settings
        Vec3D lookfrom(278, 278, -800);
        Vec3D lookat(278, 278, 0);
        Vec3D vup(0,1,0);
        float vfov = 60.0;
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

__global__ void free_world(Primitive **d_list, Primitive **d_world, camera **d_camera, int num_of_primitives_to_delete) {
    for(int i=0; i < num_of_primitives_to_delete; i++) {
        delete ((Sphere *)d_list[i])->mat_ptr;
        delete d_list[i];
    }
    delete *d_world;
    delete *d_camera;
}

__global__ void free_Cornell_Box_world(Primitive **d_list, Primitive **d_world, camera **d_camera, int num_of_primitives_to_delete) {
    if (threadIdx.x == 0 && blockIdx.x == 0) {
        delete* d_world;
        delete* d_camera;
    }
}

/*
void Cornell_Box_scene() {
    int nx = 800;
    int ny = 800;

    int ns = 10;

    int tx = 8;
    int ty = 8;

    std::cerr << "Rendering a " << nx << "x" << ny << " image with " << ns << " samples per pixel " <<
              "in " << tx << "x" << ty << " blocks.\n";

    int num_pixels = ns * ny;
    size_t fb_size = num_pixels * sizeof(Vec3D);

    // Allocate fb
    Vec3D *fb;
    checkCudaErrors(cudaMallocManaged((void **)&fb, fb_size));


    // Allocate first random state
    curandState *d_rand_state;
    checkCudaErrors(cudaMalloc((void **)&d_rand_state, num_pixels * sizeof(curandState)));

    // Allocate second random state
    curandState *d_rand_state2;
    checkCudaErrors(cudaMalloc((void **)&d_rand_state2, 1 * sizeof(curandState)));

    // Initialize second random state for world creation
    rand_init<<<1,1>>>(d_rand_state2);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    // Make our world
    Primitive **d_list;
    int num_primitives = 22*22+1+3;
    checkCudaErrors(cudaMalloc((void **)&d_list, num_primitives * sizeof(Primitive *)));

    Primitive **d_world;
    checkCudaErrors(cudaMalloc((void **)&d_world, sizeof(Primitive *)));

    camera **d_camera;
    checkCudaErrors(cudaMalloc((void **)&d_camera, sizeof(camera *)));
    create_Cornell_Box_world<<<1,1>>>(d_list, d_world, d_camera, nx, ny, d_rand_state2);
    create_world<<<1,1>>>(d_list, d_world, d_camera, nx, ny, d_rand_state2);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    // Start time measurement
    clock_t start = clock();

    // Render the buffer
    dim3 blocks(nx/tx+1, ny/ty+1);
    dim3 threads(tx, ty);
    render_init<<<blocks, threads>>>(nx, ny, d_rand_state);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());
    render<<<blocks, threads>>>(fb, nx, ny, ns, d_camera, d_world, d_rand_state);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    // End time measurement
    clock_t end = clock();
    double time_spent = ((double)(end - start)) / CLOCKS_PER_SEC;
    std::cerr << "took " << time_spent << " seconds.\n";

    // Write to the .ppm file
    const std::string& file_name = "GPU_image_Cornell.ppm";
    std::ofstream ofs(file_name, std::ios_base::out | std::ios_base::binary);

    ofs << "P3\n" << nx << " " << ny << "\n255\n";
    for (int j = ny-1; j >= 0; j--) {
        for (int i = 0; i < nx; i++) {
            size_t pixel_index = j*nx + i;
            int ir = int(255.99*fb[pixel_index].x());
            int ig = int(255.99*fb[pixel_index].y());
            int ib = int(255.99*fb[pixel_index].z());
            ofs << ir << " " << ig << " " << ib << "\n";
        }
    }

    ofs.close();

    // Clean resources
    checkCudaErrors(cudaDeviceSynchronize());
    free_world<<<1,1>>>(d_list,d_world,d_camera, num_primitives);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaFree(d_camera));
    checkCudaErrors(cudaFree(d_world));
    checkCudaErrors(cudaFree(d_list));
    checkCudaErrors(cudaFree(d_rand_state));
    checkCudaErrors(cudaFree(d_rand_state2));
    checkCudaErrors(cudaFree(fb));

    cudaDeviceReset();

}
*/
void final_scene() {
    cudaDeviceSetLimit(cudaLimitStackSize, 65536);
    int nx = 1200;
    int ny = 800;
    int ns = 30;
    int tx = 8;
    int ty = 8;

    std::cerr << "Rendering a " << nx << "x" << ny << " image with " << ns << " samples per pixel ";
    std::cerr << "in " << tx << "x" << ty << " blocks.\n";

    int num_pixels = nx*ny;
    size_t fb_size = num_pixels*sizeof(Vec3D);

    // allocate FB
    Vec3D *fb;
    checkCudaErrors(cudaMallocManaged((void **)&fb, fb_size));

    // allocate random state
    curandState *d_rand_state;
    checkCudaErrors(cudaMalloc((void **)&d_rand_state, num_pixels*sizeof(curandState)));
    curandState *d_rand_state2;
    checkCudaErrors(cudaMalloc((void **)&d_rand_state2, 1*sizeof(curandState)));

    // we need that 2nd random state to be initialized for the world creation
    rand_init<<<1,1>>>(d_rand_state2);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    // make our world of hitables & the camera
    Primitive **d_list;
    int num_hitables = 22*22+1+3+1;
    checkCudaErrors(cudaMalloc((void **)&d_list, num_hitables*sizeof(Primitive *)));
    Primitive **d_world;
    checkCudaErrors(cudaMalloc((void **)&d_world, sizeof(Primitive *)));
    camera **d_camera;
    checkCudaErrors(cudaMalloc((void **)&d_camera, sizeof(camera *)));
    create_world<<<1,1>>>(d_list, d_world, d_camera, nx, ny, d_rand_state2);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    clock_t start, stop;
    start = clock();
    // Render our buffer
    dim3 blocks(nx/tx+1,ny/ty+1);
    dim3 threads(tx,ty);
    render_init<<<blocks, threads>>>(nx, ny, d_rand_state);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());
    render<<<blocks, threads>>>(fb, nx, ny,  ns, d_camera, d_world, d_rand_state);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());
    stop = clock();
    double timer_seconds = ((double)(stop - start)) / CLOCKS_PER_SEC;
    std::cerr << "took " << timer_seconds << " seconds.\n";

    const std::string& file_name = "GPU_image_new.ppm";
    std::ofstream ofs(file_name, std::ios_base::out | std::ios_base::binary);

    ofs << "P3\n" << nx << " " << ny << "\n255\n";
    for (int j = ny-1; j >= 0; j--) {
        for (int i = 0; i < nx; i++) {
            size_t pixel_index = j*nx + i;
            int ir = int(255.99*fb[pixel_index].x());
            int ig = int(255.99*fb[pixel_index].y());
            int ib = int(255.99*fb[pixel_index].z());
            ofs << ir << " " << ig << " " << ib << "\n";
        }
    }

    ofs.close();

    // clean up
    checkCudaErrors(cudaDeviceSynchronize());
    free_world<<<1,1>>>(d_list,d_world,d_camera, 22*22+1+3);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaFree(d_camera));
    checkCudaErrors(cudaFree(d_world));
    checkCudaErrors(cudaFree(d_list));
    checkCudaErrors(cudaFree(d_rand_state));
    checkCudaErrors(cudaFree(d_rand_state2));
    checkCudaErrors(cudaFree(fb));

    cudaDeviceReset();
}

void test() {
    cudaDeviceSetLimit(cudaLimitStackSize, 65536);
    int nx = 800;
    int ny = 800;
    int ns = 10;
    int tx = 8;        // num of threads_x
    int ty = 8;       // num of threads_y

    /*
     *  dim3 blocks(nx/tx+1,ny/ty+1);
    dim3 threads(tx,ty);

     */

    std::cerr << "Rendering a " << nx << "x" << ny << " image with " << ns << " samples per pixel ";
    std::cerr << "in " << tx << "x" << ty << " blocks.\n";

    int num_pixels = nx*ny;
    size_t fb_size = num_pixels*sizeof(Vec3D);

    // allocate FB
    Vec3D *fb;
    checkCudaErrors(cudaMallocManaged((void **)&fb, fb_size));

    // allocate random state
    curandState *d_rand_state;
    checkCudaErrors(cudaMalloc((void **)&d_rand_state, num_pixels*sizeof(curandState)));
    curandState *d_rand_state2;
    checkCudaErrors(cudaMalloc((void **)&d_rand_state2, 1*sizeof(curandState)));

    // we need that 2nd random state to be initialized for the world creation
    rand_init<<<1,1>>>(d_rand_state2);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    // make our world of hitables & the camera
    // NEW: ADD BUNNY
    int num_vertices = 0;
    int num_faces = 0;
    get_num_vertices_and_faces("C:\\Users\\ramik\\OneDrive\\Desktop\\Folders\\Models\\teapot.obj", num_vertices, num_faces);

    auto *vertices = (Vec3D*)calloc(num_vertices, sizeof(Vec3D));

    auto* faces = (Vec3D*)calloc(num_faces*3, sizeof(Vec3D));

    Vec3D displacement(400.0, -0.5, 210.0);
    float scale_factor = 40.0f;
    float angle_of_rotation = -59.5f;

    load_model("C:\\Users\\ramik\\OneDrive\\Desktop\\Folders\\Models\\teapot.obj", vertices, faces, displacement, scale_factor, angle_of_rotation);

    // TODO: send faces to kernel function: create_Cornell_Box_world<<<1,1>>>(faces, d_list, d_world, d_camera, nx, ny, d_rand_state2);
    // CUDA kernels operate on GPU memory, so let's copy the data from the CPU to the GPU before using it in the kernel

    printf("HOST: num_faces = %d\n", num_faces);
    std::cout << "HOST: ";
    printf("%f\n", faces[0].x());
    std::cout << "HOST: ";
    printf("%f\n", faces[3].y());

    // 1. Allocate memory on the device for faces
    Vec3D* d_faces;
    checkCudaErrors(cudaMalloc((void**)&d_faces, num_faces * 3 * sizeof(Vec3D)));

    // 2. Copy faces from the host to the device:
    checkCudaErrors(cudaMemcpy(d_faces, faces, num_faces * 3 * sizeof(Vec3D), cudaMemcpyHostToDevice));

    // END NEW
    Primitive **d_list;
    int num_hitables = 8+num_faces;// num_faces+1
    checkCudaErrors(cudaMalloc((void **)&d_list, num_hitables*sizeof(Primitive *)));
    Primitive **d_world;
    checkCudaErrors(cudaMalloc((void **)&d_world, sizeof(Primitive *)));
    camera **d_camera;
    checkCudaErrors(cudaMalloc((void **)&d_camera, sizeof(camera *)));
    create_Cornell_Box_world<<<1,1>>>(d_faces, num_faces, d_list, d_world, d_camera, nx, ny, d_rand_state2);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    clock_t start, stop;
    start = clock();
    // Render our buffer
    dim3 blocks(nx/tx+1,ny/ty+1);
    dim3 threads(tx,ty);
    render_init<<<blocks, threads>>>(nx, ny, d_rand_state);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());
    render<<<blocks, threads>>>(fb, nx, ny,  ns, d_camera, d_world, d_rand_state);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());
    stop = clock();
    double timer_seconds = ((double)(stop - start)) / CLOCKS_PER_SEC;
    std::cerr << "took " << timer_seconds << " seconds.\n";

    const std::string& file_name = "bobo.ppm";
    std::ofstream ofs(file_name, std::ios_base::out | std::ios_base::binary);

    ofs << "P3\n" << nx << " " << ny << "\n255\n";
    for (int j = ny-1; j >= 0; j--) {
        for (int i = 0; i < nx; i++) {
            size_t pixel_index = j*nx + i;
            int ir = int(255.99*fb[pixel_index].x());
            int ig = int(255.99*fb[pixel_index].y());
            int ib = int(255.99*fb[pixel_index].z());
            ofs << ir << " " << ig << " " << ib << "\n";
        }
    }

    ofs.close();

    // clean up
    checkCudaErrors(cudaDeviceSynchronize());
    free_Cornell_Box_world<<<1,1>>>(d_list,d_world,d_camera, 8+num_faces);
    checkCudaErrors(cudaFree(d_camera));
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaFree(d_world));
    checkCudaErrors(cudaFree(d_list));
    checkCudaErrors(cudaFree(d_rand_state));
    checkCudaErrors(cudaFree(d_rand_state2));
    checkCudaErrors(cudaFree(fb));

    cudaDeviceReset();
}
int main() {
    // final_scene();

    //  Cornell_Box_scene();
    //final_scene();
    int deviceID = 0; // You can change this to the desired CUDA device ID
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


    //final_scene();


    test();
    int num_verts = 0;
    int num_fcs = 0;
    get_num_vertices_and_faces("C:\\Users\\ramik\\OneDrive\\Desktop\\Folders\\Models\\Tetrahedron.obj.txt", num_verts, num_fcs);
    std::cout << num_fcs << std::endl;
}