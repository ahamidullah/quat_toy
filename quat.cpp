#include <SDL2/SDL.h>
#include <math.h>
#include <stdio.h>

struct Vec2i {
	int x, y;
};

struct Vec3i {
	int x, y, z;
};

struct Vec3f {
	float x, y, z;
	inline Vec3f &operator+=(Vec3f v);
	inline Vec3f &operator-=(Vec3f v);
};

struct Vec2f {
	float x, y;
};

inline Vec3f
operator/(Vec3f v, float divisor)
{
	Vec3f res;
	res.x = v.x / divisor;
	res.y = v.y / divisor;
	res.z = v.z / divisor;
	return res;
}

inline Vec3f
operator*(float a, Vec3f b)
{
	Vec3f result;
	result.x = b.x * a;
	result.y = b.y * a;
	result.z = b.z * a;
	return result;
}

inline Vec3f
operator-(Vec3f a, Vec3f b)
{
	Vec3f result;
	result.x = a.x - b.x;
	result.y = a.y - b.y;
	result.z = a.z - b.z;
	return result;
}

inline Vec3f
operator+(Vec3f a, Vec3f b)
{
	Vec3f result;
	result.x = a.x + b.x;
	result.y = a.y + b.y;
	result.z = a.z + b.z;
	return result;
}

inline Vec3f &Vec3f::
operator-=(Vec3f v)
{
	*this = *this - v;
	return *this;
}

inline Vec3f &Vec3f::
operator+=(Vec3f v)
{
	*this = *this + v;
	return *this;
}

inline Vec3f
operator-(Vec3f v)
{
	return -1.0f * v;
}

inline float
dot_product(Vec3f a, Vec3f b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

inline Vec3f
cross_product(Vec3f a, Vec3f b)
{
	return { a.y*b.z - b.y*a.z,
	         a.z*b.x - b.z*a.x,
	         a.x*b.y - b.x*a.y };
}

struct Quaternion {
	float w;
	Vec3f im;
};

inline Quaternion
operator*(Quaternion q1, Quaternion q2)
{
	return { q1.w*q2.w - dot_product(q1.im, q2.im),
	         q1.w*q2.im + q2.w*q1.im + cross_product(q1.im, q2.im) };
}

inline Quaternion
operator/(Quaternion q, float divisor)
{
	return { q.w / divisor, q.im / divisor };
}

inline Quaternion
make_quat()
{
	return { 0.0f, {1.0f, 1.0f, 1.0f} };
}

inline Quaternion
make_quat(Vec3f v)
{
	return { 0.0f, v };
}

inline Quaternion
make_quat(Quaternion q, float radians, Vec3f axis)
{
	float half_rads = radians / 2.0f;
	return (Quaternion){ cos(half_rads), sin(half_rads) * axis } * q;
}

inline Quaternion
normalize(Quaternion q)
{
	return q / sqrt(q.w*q.w + dot_product(q.im, q.im));
}

inline Quaternion
conjugate_quat(Quaternion q)
{
	return { q.w, -q.im };
}

inline Vec3f
rotate_vector(Vec3f v, Quaternion q)
{
	Quaternion res = q * make_quat(v) * conjugate_quat(q);
	return res.im;
}

int
main(int, char **)
{
	constexpr int screen_w = 1200, screen_h = 800;
	SDL_Window *window;
	SDL_Renderer *renderer;
	if (SDL_Init(SDL_INIT_VIDEO) < 0) {
		printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
		return 1;
	}
	window = SDL_CreateWindow("Quat_Toy", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, screen_w, screen_h, SDL_WINDOW_SHOWN);
	if(window == NULL) {
		printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
		return 1;
	}
	renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
	if(!renderer)
		printf("Renderer could not be created! SDL Error: %s\n", SDL_GetError());

	// Top left is (0, 0).
	constexpr float cube_len = 10.0f;
	constexpr float cube_half_len = cube_len/2.0f;
	constexpr Vec2i screen_mid = { screen_w/2, screen_h/2 };
	constexpr float near_plane = 1.0f; 

	Vec3f cube_verts[] = {
		{ -cube_half_len,  cube_half_len,  cube_half_len },
		{  cube_half_len,  cube_half_len,  cube_half_len },
		{  cube_half_len,  cube_half_len, -cube_half_len },
		{ -cube_half_len,  cube_half_len, -cube_half_len },
		{ -cube_half_len, -cube_half_len,  cube_half_len },
		{  cube_half_len, -cube_half_len,  cube_half_len },
		{  cube_half_len, -cube_half_len, -cube_half_len },
		{ -cube_half_len, -cube_half_len, -cube_half_len }, };

	float fov = 90.0f;
	float top = tan((fov / 2.0f) * (M_PI / 180.0f)) * near_plane;
	float right = top * ((float)screen_w / screen_h);

	SDL_Event event;
	bool running = true;
	bool paused = false;

	while (running) {
		while(SDL_PollEvent(&event) != 0) {
			if(event.type == SDL_QUIT) running = false;
			if(event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_p) paused = !paused;
		}

		SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0x00, 0x00);
		SDL_RenderClear(renderer);

		SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, 0xFF);
		SDL_RenderDrawPoint(renderer, screen_mid.x, screen_mid.y);

		Vec2f screen[8];
		if (!paused) {
			Quaternion rot_quat = make_quat(make_quat(), 0.001f, {1.0f, 1.0f, 0.0f});
			rot_quat = normalize(rot_quat); // Avoid accumulated round-off error.
			for (int i = 0; i < 8; ++i) {
				// Model transform (rotation only).
				cube_verts[i] = rotate_vector(cube_verts[i], rot_quat);
				// View space (looking down positive z).
				float w = cube_verts[i].z + cube_len + near_plane + 10.0f;
				// Projected point.
				screen[i].x = round(((cube_verts[i].x / w) + right) / (2*right) * screen_w);
				screen[i].y = round(((cube_verts[i].y / w) + top) / (2*top) * screen_h);
			}
		}

		SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, 0xFF);
		SDL_RenderDrawLine(renderer, screen[0].x, screen[0].y, screen[1].x, screen[1].y);
		SDL_RenderDrawLine(renderer, screen[1].x, screen[1].y, screen[2].x, screen[2].y);
		SDL_RenderDrawLine(renderer, screen[2].x, screen[2].y, screen[3].x, screen[3].y);
		SDL_RenderDrawLine(renderer, screen[3].x, screen[3].y, screen[0].x, screen[0].y);

		SDL_RenderDrawLine(renderer, screen[4].x, screen[4].y, screen[5].x, screen[5].y);
		SDL_RenderDrawLine(renderer, screen[5].x, screen[5].y, screen[6].x, screen[6].y);
		SDL_RenderDrawLine(renderer, screen[6].x, screen[6].y, screen[7].x, screen[7].y);
		SDL_RenderDrawLine(renderer, screen[7].x, screen[7].y, screen[4].x, screen[4].y);

		SDL_RenderDrawLine(renderer, screen[0].x, screen[0].y, screen[4].x, screen[4].y);
		SDL_RenderDrawLine(renderer, screen[1].x, screen[1].y, screen[5].x, screen[5].y);
		SDL_RenderDrawLine(renderer, screen[2].x, screen[2].y, screen[6].x, screen[6].y);
		SDL_RenderDrawLine(renderer, screen[3].x, screen[3].y, screen[7].x, screen[7].y);

		SDL_RenderPresent(renderer);
	}

	SDL_DestroyWindow(window);
	SDL_Quit();
	return 0;
}

