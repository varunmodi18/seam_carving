# Seam Carving - Content-Aware Image Resizing

A C++ implementation of the seam carving algorithm for intelligent image resizing. This program removes pixels along paths of least importance (seams) to reduce image dimensions while preserving important visual content.

## What is Seam Carving?

Seam carving is a content-aware image resizing technique that intelligently removes low-importance pixel paths instead of uniformly scaling the image. This preserves the most visually significant parts of the image.

## Installation

### Dependencies

Install OpenCV development libraries:

```bash
sudo apt update
sudo apt install build-essential libopencv-dev
```

### Building

**Option 1: Using CMake**
```bash
mkdir build && cd build
cmake ..
make
```

**Option 2: Manual Compilation**
```bash
g++ src/seam_carving.cpp -o seam_carving `pkg-config --cflags --libs opencv4` -std=c++17
```

## How to Run

```bash
./opencv_vscode
```

The program will prompt you to:
1. Enter the image path
2. Specify target width and height
3. Watch the real-time visualization of seams being removed
4. Output is saved as `output.png`

**Example:**
```
Enter complete path of the image: sample_input/sample1.jpeg
Current image dimensions are: 1920x1079
Please specify the new dimensions,
Width: 1600
Height: 900
```

## Implementation Details

### 1. Dual Gradient Energy Function

The algorithm calculates the "importance" of each pixel based on color gradients:

```cpp
Energy dual_gradient_energy(const Cube& cube, size_t height, size_t width, size_t depth) {
    Energy energy(height, width);
    
    for each pixel (x, y):
        // Calculate horizontal gradient
        Δx² = (B_right - B_left)² + (G_right - G_left)² + (R_right - R_left)²
        
        // Calculate vertical gradient
        Δy² = (B_down - B_up)² + (G_down - G_up)² + (R_down - R_up)²
        
        // Total energy
        energy(x, y) = Δx² + Δy²
    
    return energy;
}
```

**Key points:**
- Uses adjacent pixels with wraparound (modulo arithmetic for edges)
- Higher gradient = more important pixel
- Calculates gradients across all 3 color channels (BGR)

### 2. Seam Finding (Dynamic Programming)

Finds the minimum-energy connected path through the image:

```cpp
size_t* find_vertical_seam(const Energy& energy, size_t height, size_t width) {
    // Initialize DP arrays
    double dist[height][width];  // Cumulative minimum energy
    int back[height][width];     // Backtracking pointers
    
    // Base case: first row
    dist[0][x] = energy(0, x) for all x
    
    // Fill DP table
    for y = 1 to height:
        for x = 0 to width:
            // Choose minimum from three predecessors
            dist[y][x] = energy(y, x) + min(
                dist[y-1][x-1],  // diagonal left
                dist[y-1][x],    // directly above
                dist[y-1][x+1]   // diagonal right
            )
            back[y][x] = x_of_minimum_predecessor
    
    // Find minimum cost in last row
    // Backtrack to reconstruct seam path
}
```

**Complexity:**
- Time: O(width × height)
- Space: O(width × height)

### 3. Seam Removal

Removes pixels along the identified seam:

```cpp
void delete_vertical_seam(Cube &cube, const size_t* seam, size_t &height, size_t &width) {
    for each row y:
        x = seam[y]  // Column to remove in this row
        
        // Shift all pixels after x to the left
        for j = x to width-2:
            cube(y, j, channel) = cube(y, j+1, channel) for all channels
    
    width -= 1  // Image is now 1 pixel narrower
}
```

### 4. Iteration Process

The algorithm repeats until target dimensions are reached:

1. **Remove vertical seams** (reduce width):
   - While current_width > target_width:
     - Calculate energy map
     - Find optimal vertical seam
     - Remove seam
     - Update width

2. **Remove horizontal seams** (reduce height):
   - While current_height > target_height:
     - Calculate energy map
     - Find optimal horizontal seam
     - Remove seam
     - Update height

### Data Structures

**Cube Class** (3D array for BGR images):
```cpp
class Cube {
    unsigned char* data;
    size_t height, width, depth;
    
    // Access pixel: cube(y, x, channel)
    // Memory layout: row-major, channel-last
};
```

**Energy Class** (2D array for energy values):
```cpp
class Energy {
    double* data;
    size_t height, width;
    
    // Access energy: energy(y, x)
};
```

## Algorithm Flow

```
1. Load image into Cube data structure
2. While dimensions > target:
   a. Calculate energy map using dual gradient
   b. Find minimum-energy seam using DP
   c. Remove seam and shift pixels
   d. Display result (optional)
3. Save final resized image
```

## Project Structure

```
seam_carving/
├── CMakeLists.txt           # Build configuration
├── README.md                # This file
├── src/
│   └── seam_carving.cpp    # Main implementation
└── sample_input/           # Test images
    ├── sample1.jpeg
    ├── sample2.jpeg
    └── ...
```
