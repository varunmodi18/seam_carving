//====================================================================================================
//                    HEADER FILES
//====================================================================================================

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <cstddef>
using namespace std;

//const int MOD = 1e9 + 7;





//====================================================================================================
//                    3-D ARRAY FOR BGR IMAGE
//====================================================================================================

class Cube {
    unsigned char* data;
    size_t height, width, depth;

public:
    // initialiser
    // Using size_t ensures the multiplication is done in a size-safe type, preventing overflow on large images.
    Cube(size_t h, size_t w, size_t d) : height(h), width(w), depth(d) {
        // Allocate memory for the image data in a flattened 3D array.
        // Total size = height × width × depth bytes.
        // Each element (unsigned char) stores one channel value in [0, 255].
        data = new unsigned char[h * w * d];
    }

    // free the heap memory when the object is deleted.
    ~Cube() { delete[] data; }

    unsigned char& operator()(size_t y, size_t x, size_t c) {
        return data[(y * width + x) * depth + c];
    }

    const unsigned char& operator()(size_t y, size_t x, size_t c) const {
        return data[(y * width + x) * depth + c];
    }
};





//====================================================================================================
//                    2-D ARRAY FOR ENERGY
//====================================================================================================

class Energy {
    double* data;
    size_t height, width;

public:
    // initialiser
    // Using size_t ensures the multiplication is done in a size-safe type, preventing overflow on large images.
    Energy(size_t h, size_t w) : height(h), width(w) {
        // Allocate memory for the image data in a flattened 2D array.
        // Total size = height × width bytes.
        // Each element (unsigned char) stores one channel value in [0, 255].
        data = new double[h * w];
    }

    // free the heap memory when the object is deleted.
    ~Energy() { delete[] data; }

    double& operator()(size_t y, size_t x) {
        return data[y * width + x];
    }

    const double& operator()(size_t y, size_t x) const {
        return data[y * width + x];
    }
};





//====================================================================================================
//                    FUNCTION TO CALCULATE ENERGY
//====================================================================================================

Energy dual_gradient_energy(const Cube& cube, size_t height, size_t width, size_t depth) {
    Energy energy(height, width);

    // loop over rows
    for (size_t row_number = 0; row_number < height; row_number++) {
        size_t upper_pixel = (row_number + height - 1) % height; 
        size_t lower_pixel = (row_number + 1) % height;     

        // loop over columns
        for (size_t column_number = 0; column_number < width; column_number++) {
            size_t left_pixel = (column_number + width - 1) % width; 
            size_t right_pixel = (column_number + 1) % width;     

            // cast to int to avoid unsigned underflow; square in 64-bit
            int bx = int(cube(row_number, right_pixel, 0)) - int(cube(row_number, left_pixel, 0));
            int gx = int(cube(row_number, right_pixel, 1)) - int(cube(row_number, left_pixel, 1));
            int rx = int(cube(row_number, right_pixel, 2)) - int(cube(row_number, left_pixel, 2));
            long long dx2 = 1LL*bx*bx + 1LL*gx*gx + 1LL*rx*rx;

            int by = int(cube(lower_pixel, column_number, 0)) - int(cube(upper_pixel, column_number, 0));
            int gy = int(cube(lower_pixel, column_number, 1)) - int(cube(upper_pixel, column_number, 1));
            int ry = int(cube(lower_pixel, column_number, 2)) - int(cube(upper_pixel, column_number, 2));
            long long dy2 = 1LL*by*by + 1LL*gy*gy + 1LL*ry*ry;

            energy(row_number, column_number) = double(dx2 + dy2);
        }
    }
    return energy; // caller: delete[] energy;
}





//====================================================================================================
//                    FUNCTION TO CALCULATE VERTICAL SEAM
//====================================================================================================

size_t* find_vertical_seam(const Energy& energy, size_t height, size_t width) {
    // DP buffers (row-major)
    double* dist = new double[height * width];
    int*    back = new int[height * width];   // previous column index for path

    // init first row
    for (size_t column_number = 0; column_number < width; column_number++) {
        dist[0 * width + column_number] = energy(0, column_number);
        back[0 * width + column_number] = -1;        // start of seam
    }

    // fill DP
    for (size_t row_number = 1; row_number < height; row_number++) {
        for (size_t column_number = 0; column_number < width; column_number++) {
            // choose best predecessor among (y-1, x-1), (y-1, x), (y-1, x+1)
            double best_val = dist[(row_number - 1) * width + column_number];
            int    best_x   = (int)column_number;

            if (column_number > 0 && dist[(row_number - 1) * width + (column_number - 1)] < best_val) {
                best_val = dist[(row_number - 1) * width + (column_number - 1)];
                best_x   = (int)column_number - 1;
            }
            if (column_number + 1 < width && dist[(row_number - 1) * width + (column_number + 1)] < best_val) {
                best_val = dist[(row_number - 1) * width + (column_number + 1)];
                best_x   = (int)column_number + 1;
            }

            dist[row_number * width + column_number] = best_val + energy(row_number, column_number);
            back[row_number * width + column_number] = best_x;
        }
    }

    // find min end in last row
    size_t best_col = 0;
    double best_sum = dist[(height - 1) * width + 0];
    for (size_t column_number = 1; column_number < width; column_number++) {
        double v = dist[(height - 1) * width + column_number];
        if (v < best_sum) { best_sum = v; best_col = column_number; }
    }

    // reconstruct seam (bottom -> top)
    size_t* seam = new size_t[height];
    seam[height - 1] = best_col;
    for (size_t row_number = height - 1; row_number > 0; row_number--) {
        int prev_column_number = back[row_number * width + seam[row_number]];
        seam[row_number - 1] = (size_t)prev_column_number;
    }

    delete[] dist;
    delete[] back;
    return seam; // caller: delete[] seam;
}





//====================================================================================================
//                    FUNCTION TO CALCULATE HORIZONTAL SEAM
//====================================================================================================

size_t* find_horizontal_seam(const Energy& energy, size_t height, size_t width) {
    // DP buffers
    double* dist = new double[height * width];
    int*    back = new int[height * width];   // previous row index for path

    // init first column
    for (size_t row_number = 0; row_number < height; row_number++) {
        dist[row_number * width + 0] = energy(row_number, 0);
        back[row_number * width + 0] = -1;
    }

    // fill DP left->right
    for (size_t column_number = 1; column_number < width; column_number++) {
        for (size_t row_number = 0; row_number < height; ++row_number) {
            // predecessors: (y-1,x-1), (y,x-1), (y+1,x-1)
            double best_val = dist[row_number * width + (column_number - 1)];
            int    best_y   = (int)row_number;

            if (row_number > 0 && dist[(row_number - 1) * width + (column_number - 1)] < best_val) {
                best_val = dist[(row_number - 1) * width + (column_number - 1)];
                best_y   = (int)row_number - 1;
            }
            if (row_number + 1 < height && dist[(row_number + 1) * width + (column_number - 1)] < best_val) {
                best_val = dist[(row_number + 1) * width + (column_number - 1)];
                best_y   = (int)row_number + 1;
            }

            dist[row_number * width + column_number] = best_val + energy(row_number, column_number);
            back[row_number * width + column_number] = best_y;
        }
    }

    // min end in last column
    size_t best_row = 0;
    double best_sum = dist[0 * width + (width - 1)];
    for (size_t row_number = 1; row_number < height; row_number++) {
        double v = dist[row_number * width + (width - 1)];
        if (v < best_sum) { best_sum = v; best_row = row_number; }
    }

    // reconstruct seam (right -> left)
    size_t* seam = new size_t[width];
    seam[width - 1] = best_row;
    for (size_t column_number = width - 1; column_number > 0; column_number--) {
        int prev_y = back[seam[column_number] * width + column_number];
        seam[column_number - 1] = (size_t)prev_y;
    }

    delete[] dist;
    delete[] back;
    return seam; // caller: delete[] seam;
}





//====================================================================================================
//                    FUNCTIONS TO PLOT IMAGE WITH SEAM MARKED
//====================================================================================================

enum class SeamDir { Vertical, Horizontal };

// Option A: explicit orientation
void overlaySeamRed(Cube &cube,
                    const size_t* seam,
                    size_t height, size_t width,
                    SeamDir dir)
{
    auto paint = [&](size_t y, size_t x){
        // center
        cube(y, x, 0) = 0;   // B
        cube(y, x, 1) = 0;   // G
        cube(y, x, 2) = 255; // R
    };

    if (dir == SeamDir::Vertical) {
        // seam[y] == x for each row y
        for (size_t y = 0; y < height; ++y) {
            size_t x = seam[y];
            if (x >= width) continue; // safety
            paint(y, x);
            // left/right neighbors to make it visible
            if (x > 0)        paint(y, x - 1);
            if (x + 1 < width) paint(y, x + 1);
        }
    } else {
        // Horizontal: seam[x] == y for each column x
        for (size_t x = 0; x < width; ++x) {
            size_t y = seam[x];
            if (y >= height) continue; // safety
            paint(y, x);
            // up/down neighbors
            if (y > 0)          paint(y - 1, x);
            if (y + 1 < height) paint(y + 1, x);
        }
    }
}

cv::Mat cubeToMat(const Cube& cube, size_t height, size_t width) {
    cv::Mat img((int)height, (int)width, CV_8UC3);
    for (size_t y = 0; y < height; ++y) {
        cv::Vec3b* row = img.ptr<cv::Vec3b>((int)y);
        for (size_t x = 0; x < width; ++x) {
            row[(int)x][0] = cube(y, x, 0); // B
            row[(int)x][1] = cube(y, x, 1); // G
            row[(int)x][2] = cube(y, x, 2); // R
        }
    }
    return img;
}





//====================================================================================================
//                    FUNCTIONS TO DELETE SEAM
//====================================================================================================

// Delete a vertical seam: seam[y] = x
void delete_vertical_seam(Cube &cube, const size_t* seam, size_t &height, size_t &width) {
    for (size_t y = 0; y < height; ++y) {
        size_t x = seam[y];
        if (x >= width) continue; // safety
        // Shift all pixels after seam left by 1
        for (size_t j = x; j < width - 1; ++j) {
            cube(y, j, 0) = cube(y, j + 1, 0);
            cube(y, j, 1) = cube(y, j + 1, 1);
            cube(y, j, 2) = cube(y, j + 1, 2);
        }
    }
    width -= 1; // image is now 1 column smaller
}

// Delete a horizontal seam: seam[x] = y
void delete_horizontal_seam(Cube &cube, const size_t* seam, size_t &height, size_t &width) {
    for (size_t x = 0; x < width; ++x) {
        size_t y = seam[x];
        if (y >= height) continue; // safety
        // Shift all pixels after seam up by 1
        for (size_t i = y; i < height - 1; ++i) {
            cube(i, x, 0) = cube(i + 1, x, 0);
            cube(i, x, 1) = cube(i + 1, x, 1);
            cube(i, x, 2) = cube(i + 1, x, 2);
        }
    }
    height -= 1; // image is now 1 row smaller
}





//====================================================================================================
//                    FUNCTIONS TO DISPLAY THE SEAMS
//====================================================================================================

void show_with_vertical_seam(const Cube& cube,
                             size_t H, size_t W,
                             const size_t* seam,
                             const char* windowName)
{
    cv::Mat vis = cubeToMat(cube, H, W);
    for (size_t y = 0; y < H; ++y) {
        size_t x = seam[y];
        if (x >= W) continue;
        vis.at<cv::Vec3b>(static_cast<int>(y), static_cast<int>(x)) = cv::Vec3b(0, 0, 255);
        if (x > 0)
            vis.at<cv::Vec3b>(static_cast<int>(y), static_cast<int>(x - 1)) = cv::Vec3b(0, 0, 255);
        if (x + 1 < W)
            vis.at<cv::Vec3b>(static_cast<int>(y), static_cast<int>(x + 1)) = cv::Vec3b(0, 0, 255);
    }
    cv::imshow(windowName, vis);
    cv::waitKey(100);
}

void show_with_horizontal_seam(const Cube& cube,
                               size_t H, size_t W,
                               const size_t* seam,
                               const char* windowName)
{
    cv::Mat vis = cubeToMat(cube, H, W);
    for (size_t x = 0; x < W; ++x) {
        size_t y = seam[x];
        if (y >= H) continue;
        vis.at<cv::Vec3b>(static_cast<int>(y), static_cast<int>(x)) = cv::Vec3b(0, 0, 255);
        if (y > 0)
            vis.at<cv::Vec3b>(static_cast<int>(y - 1), static_cast<int>(x)) = cv::Vec3b(0, 0, 255);
        if (y + 1 < H)
            vis.at<cv::Vec3b>(static_cast<int>(y + 1), static_cast<int>(x)) = cv::Vec3b(0, 0, 255);
    }
    cv::imshow(windowName, vis);
    cv::waitKey(100);
}





//====================================================================================================
//                    MAIN FUNCTION
//====================================================================================================

int main() {
    //ios_base::sync_with_stdio(0);
    //cin.tie(NULL);
    //cout.tie(NULL);

    cout << "Enter complete path of the image: ";

    string path;
    cin >> path;

    cv::Mat img = cv::imread(path , cv::IMREAD_COLOR);
    // cv::Mat is OpenCV's matrix type for images
    // cv::IMREAD_COLOR ensures we load 3 channels (BGR)

    if (img.empty()) { 
        std::cerr << "Image not found\n";
        return 0;
    }
    else {
         // cv::imshow("OpenCV Test", img);
    }

    // Image dimensions
    // - img.rows  → number of rows (image height)
    // - img.cols  → number of columns (image width)
    // - depth     → number of channels (3 for B, G, R)
    Cube cube(img.rows, img.cols, 3);

    // Memory notes:
    // - cube points to the first byte of the allocated array on the heap.
    // - Must be released later with: delete[] cube;

    size_t new_width, new_height;
    cout << "Current image dimensions are: " << img.cols << "x" << img.rows << endl;
    cout << "Please specify the new dimensions," << endl;
    cout << "Width: ";
    cin >> new_width;
    cout << "Height: ";
    cin >> new_height;
    cout << "New Dimensions: " << new_width << "x" << new_height << endl;
    cout << "Processing... Please Wait..." << endl;
  
    // loop over rows
    for (size_t row_number = 0; row_number < img.rows; row_number++) {

        // ptr<T>(y) gives a pointer to the y-th row.
        //
        // cv::Vec3b breakdown:
        //   Vec --> a fixed-size vector
        //   3   --> contains 3 elements
        //   b   --> each element is an unsigned char (uchar), i.e., range 0–255
        //
        // So, each row[x] is a cv::Vec3b with:
        //   row[column_number][0] = Blue
        //   row[column_number][1] = Green
        //   row[column_number][2] = Red

        const cv::Vec3b* row = img.ptr<cv::Vec3b>(row_number);

        // loop over columns
        for (size_t column_number = 0; column_number < img.cols; column_number++) {
            cube(row_number, column_number, 0) = row[column_number][0]; // B
            cube(row_number, column_number, 1) = row[column_number][1]; // G
            cube(row_number, column_number, 2) = row[column_number][2]; // R
        }
    }

    Energy energy = dual_gradient_energy(cube, img.rows, img.cols, 3);

    size_t H = static_cast<size_t>(img.rows);
    size_t W = static_cast<size_t>(img.cols);

    const char* kWin = "OpenCV Test";
    cv::imshow(kWin, img);
    cv::waitKey(100);

    if (new_height > H) new_height = H;
    if (new_width  > W) new_width  = W;

    while (W > new_width && W >= 2) {
    Energy energy = dual_gradient_energy(cube, H, W, 3);
    const size_t* seam = find_vertical_seam(energy, H, W);   
    show_with_vertical_seam(cube, H, W, seam, kWin);         
    delete_vertical_seam(cube, seam, H, W);                  
    cv::Mat out_after = cubeToMat(cube, H, W);
    cv::imshow(kWin, out_after);
    cv::waitKey(100);
    }

    while (H > new_height && H >= 2) {
        Energy energy = dual_gradient_energy(cube, H, W, 3);
        const size_t* seam = find_horizontal_seam(energy, H, W); 
        show_with_horizontal_seam(cube, H, W, seam, kWin);       
        delete_horizontal_seam(cube, seam, H, W);                
        cv::Mat out_after = cubeToMat(cube, H, W);
        cv::imshow(kWin, out_after);
        cv::waitKey(100);
    }


    cv::waitKey(0);
    cv::Mat final_out = cubeToMat(cube, H, W);
    cv::imwrite("output.png", final_out);


    // delete[] seam;  // if allocated with new[]

    //cv::imwrite("output.png", out);
    //delete[] seam;
    
    return 0;
}
