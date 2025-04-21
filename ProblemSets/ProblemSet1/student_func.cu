// Homework 1
// Color to Greyscale Conversion

//A common way to represent color images is known as RGBA - the color
//is specified by how much Red, Grean and Blue is in it.
//The 'A' stands for Alpha and is used for transparency, it will be
//ignored in this homework.

//Each channel Red, Blue, Green and Alpha is represented by one byte.
//Since we are using one byte for each color there are 256 different
//possible values for each color.  This means we use 4 bytes per pixel.

//Greyscale images are represented by a single intensity value per pixel
//which is one byte in size.

//To convert an image from color to grayscale one simple method is to
//set the intensity to the average of the RGB channels.  But we will
//use a more sophisticated method that takes into account how the eye 
//perceives color and weights the channels unequally.

//The eye responds most strongly to green followed by red and then blue.
//The NTSC (National Television System Committee) recommends the following
//formula for color to greyscale conversion:

//I = .299f * R + .587f * G + .114f * B

//Notice the trailing f's on the numbers which indicate that they are 
//single precision floating point constants and not double precision
//constants.

//You should fill in the kernel as well as set the block and grid sizes
//so that the entire image is processed.

#include "utils.h"

__global__
void rgba_to_greyscale(const uchar4* const rgbaImage,
                       unsigned char* const greyImage,
                       int numRows, int numCols)
{
  //TODO
  //Fill in the kernel to convert from color to greyscale
  //the mapping from components of a uchar4 to RGBA is:
  // .x -> R ; .y -> G ; .z -> B ; .w -> A
  //
  //The output (greyImage) at each pixel should be the result of
  //applying the formula: output = .299f * R + .587f * G + .114f * B;
  //Note: We will be ignoring the alpha channel for this conversion

  //First create a mapping from the 2D block and grid locations
  //to an absolute 2D location in the image, then use that to
  //calculate a 1D offset

  // const size_t tid = threadIdx.x + blockDim.x * threadIdx.y;

  const size_t tid = blockIdx.x * (blockDim.x * blockDim.y) + threadIdx.y * (blockDim.x)  + threadIdx.x;

  const size_t tid_max = numRows * numCols - 1;
  if (tid > tid_max)
  {
    return;
  }


  // std::cout << "blockIdx.x: " << blockIdx.x << std::endl;
  // printf("INDICES: %d, %d) %d, %d | %d \n", blockIdx.x, blockIdx.y, threadIdx.x, threadIdx.y, tid);

  const char R = rgbaImage[tid].x;
  const char G = rgbaImage[tid].y;
  const char B = rgbaImage[tid].z;

  greyImage[tid] = .299f * (int)R + .587f * (int)G + .114f * (int)B;
  // greyImage[tid] = (unsigned char)(.299f * (int)R + .587f * (int)G + .114f * (int)B);
  // greyImage[tid] = (unsigned char)(.299f * (uint8_t)R + .587f * (uint8_t)G + .114f * (uint8_t)B);
 
  printf("val: %c, %c, %c | %d, %d, %d \n", R, G, B, (int)R, (int)G, (int)B);
  // printf("val: %c, %c, %c | %d, %d, %d \n", R, G, B, R, G, B);
  // printf("val: %d) %c, %c, %c | %d, %d, %d \n", tid, R, G, B, (uint8_t)R, (uint8_t)G, (uint8_t)B);


}

void your_rgba_to_greyscale(const uchar4 * const h_rgbaImage, uchar4 * const d_rgbaImage,
                            unsigned char* const d_greyImage, size_t numRows, size_t numCols)
{
  /** 
   * Notes about dimensions: 
   * 
   * Going to use 1 block that is the dimension of the image in 2D. ie. Different thread per pixel.
   * 
   * Block does not require depth/dimension corresponding to image channels per pixel, since the 
   * algorithm to convert a pixel to greyscale uses all three/four channels per pixels and is not dependent on the depth
   * per pixel. ie. No iteration index required along channels of each pixel.
   * 
   * A single grid with the single block is sufficient.
   */
  // const dim3 blockSize(numRows, numCols, 1); // ie. Threads per block
  // const dim3 gridSize(1, 1, 1); // ie. Blocks per grid
  // rgba_to_greyscale<<<gridSize, blockSize>>>(d_rgbaImage, d_greyImage, numRows, numCols);


  // TODO: Is there a way to get this max value at run time than hardcoding it so that all dims of
  // block and grid can be set w.r.t that max value to avoid breaching max limits?
  // const size_t block_max_dim = 1024;


  /** 
   * Follow up notes about dimensions: 
   * Run "/usr/local/cuda/extras/demo_suite/deviceQuery" on command line to get device hardware configs
   * 
   * Max threads per block is 1024. This means multiple blocks are necessary. 
   * Also, max size per block in each dimension is (1024, 1024, 64)
   * 
   * We have 313 * 557 = 174,341 pixels to process
   * So, if a block can process 1024 pixels, we need at least 174,341 / 1024 = 170.25 = 171 blocks
   * So, let a grid of blocks of 171 be created. Can further reduce this to a square of blocks if necessary, 
   * but 171 falls under the max size of each dimension of a grid so will proceed as a single dimension grid.
   * 
   * NOTE: Not all threads in the last block will be necessary, so a conditional check will be necessary to 
   * return if total thread index becomes > total number of pixels.
   */
  const dim3 blockSize(32, 32, 1); // 1024 threads per block
  const dim3 gridSize(171, 1, 1); // 256 blocks per grid
  rgba_to_greyscale<<<gridSize, blockSize>>>(d_rgbaImage, d_greyImage, numRows, numCols);
  
  cudaDeviceSynchronize(); checkCudaErrors(cudaGetLastError());

}
