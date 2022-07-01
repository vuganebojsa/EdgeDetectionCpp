#include <iostream>
#include <stdlib.h>
#include "BitmapRawConverter.h"
#include <chrono>
#include "tbb/task_group.h"
#include "tbb/blocked_range.h"
#include "tbb/parallel_for.h"
#define __ARG_NUM__				6
#define FILTER_SIZE				3
#define THRESHOLD				128
#define CUTOFF					128

using namespace std;
using namespace chrono;
using namespace tbb;

// Prewitt operators
int *filterHor;
int *filterVer;
int filterSize;
int filterSizeDifference;
int filterHor5x5[5 * 5] = { 9, 9, 9, 9, 9, 9, 5, 5, 5, 9, -7, -3, 0, -3, -7, -7, -3, -3, -3, -7, -7, -7, -7, -7, -7 };
int filterVer5x5[5 * 5] = { 9, 9, -7, -7, -7, 9, 5, -3, -3, -7, 9, 5, 0, -3, -7, 9, 5, -3, -3, -7, 9, 9, -7, -7, -7 };

// Edge Detection operators
int edgeDistance;

/*
* @brief Sets filter for prewitt operator to a certain dimension
* @param filter filter to set
* @param size size for filter
* @param type indicator which tells if filter is horizontal or vertical (0 - hor, 1 - ver), default is 0
*/
void setFilter(int *filter[], const int size, int type = 0) {
	*filter = new int[size * size];
	filterSize = size;
	filterSizeDifference = filterSize - 2;
	int halfSize = size / 2;

	if (type == 0) {
		for (int i = 0; i < size; i++) {
			for (int j = 0; j < 2; j++) {
				for (int k = 0; k < halfSize; k++) {
					if (j == 0) (*filter)[size * i + halfSize * j + k] = -1;
					else (*filter)[size * i + halfSize * j + k] = 1;
				}
				(*filter)[size * i + halfSize * j + halfSize] = 0;
			}
		}
	}
	else {
		for (int i = 0; i < halfSize * size; i++) {
			(*filter)[i] = -1;
		}
		for (int i = 0; i < size; i++) {
			(*filter)[halfSize * size + i] = 0;
		}
		for (int i = 0; i < halfSize * size; i++) {
			(*filter)[halfSize * size + size + i] = 1;

		}
	}
}

/*
* @brief Sets the parameters for prewitt filters and edge detection distance
* 
*/
void setParameters() {
	int dimension = 0;
	do {
		cout << "Enter prewitt filter dimension (example 3, 5, 7..)" << endl;
		cin >> dimension;
	} while (dimension < 3);
	
	do {
		cout << "Enter Edge detection distance (1, 2, 3..)" << endl;
		cin >> edgeDistance;
	} while (edgeDistance < 1);

	setFilter(&filterHor, dimension, 0);
	setFilter(&filterVer, dimension, 1);
	
}

/*
* @brief Serial version of edge detection algorithm implementation using Prewitt operator
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param width image width
* @param height image height
* @startX starting index for x
* @startY starting index for y
*/
void filter_serial_prewitt(int *inBuffer, int *outBuffer, int width, int height, int startX = 1, int startY = 1, int multiplyWidth = 1)
{
	int fullFilterSize = filterSize * filterSize;
	int Gx, Gy, G;

	for (int i = startX + filterSizeDifference; i < width - filterSizeDifference; i++) {
		for (int j = startY + filterSizeDifference; j < height - filterSizeDifference; j++) {
			Gx = 0;
			Gy = 0;
			G = 0;
			for (int k = 0; k < filterSize;k++) {
				for (int c = 0; c < filterSize;c++) {
					Gx += inBuffer[(c - 1 + j) * multiplyWidth + i + k - 1] * filterHor[k * filterSize + c];
					Gy += inBuffer[(c - 1 + j) * multiplyWidth + i + k - 1] * filterVer[k * filterSize + c];
				}
			}
			G = abs(Gx) + abs(Gy);

			if (G < THRESHOLD)
				outBuffer[j * multiplyWidth + i] = 0;
			else 
				outBuffer[j * multiplyWidth + i] = 255;
		}
	}
}


/**
* @brief Parallel version of edge detection algorithm implementation using Prewitt operator
* 
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param width image width
* @param height image height
*/
void filter_parallel_prewitt(int *inBuffer, int *outBuffer, int startWidth, int startHeight, int width, int height, int multiplyWidth)
{
	task_group g;
	if (abs(height - startHeight)/2 < CUTOFF || abs(width - startWidth)/2 < CUTOFF) {
		filter_serial_prewitt(inBuffer, outBuffer, width, height, startWidth, startHeight, multiplyWidth);
	}
	else {
		g.run([&] {filter_parallel_prewitt(inBuffer, outBuffer, startWidth, startHeight, startWidth + (width - startWidth)/2 + filterSizeDifference, startHeight + (height - startHeight)/2 + filterSizeDifference, multiplyWidth); });
		g.run([&] {filter_parallel_prewitt(inBuffer, outBuffer, startWidth + (width - startWidth) / 2 - filterSizeDifference, startHeight, width, startHeight + (height - startHeight) / 2 + filterSizeDifference, multiplyWidth); });
		g.run([&] {filter_parallel_prewitt(inBuffer, outBuffer, startWidth, startHeight + (height - startHeight) / 2 - filterSizeDifference, startWidth + (width - startWidth)/2 + filterSizeDifference, height, multiplyWidth); });
		g.run([&] {filter_parallel_prewitt(inBuffer, outBuffer, startWidth + (width - startWidth)/2 - filterSizeDifference, startHeight + (height - startHeight) / 2 - filterSizeDifference, width, height, multiplyWidth); });
		g.wait(); 
	}
}


/*
@brief Check if there is a pixel around a certain pixel that is either 1 or 0 depending on the check indicator

@param i x position of selected pixel
@param j y position of selected pixel
@param width width of our image
@param inBuffer input image
@param checkIndicator indicator which tells us if we are looking for a 0 or a 1
*/
short check_around_pixel(int i, int j, int width, int* inBuffer, int checkIndicator, int multiplyWidth = 1) {

	if (multiplyWidth == 1) multiplyWidth = width;

	for (int k = -edgeDistance;k < edgeDistance  + 1;k++) {
		for (int c = -edgeDistance;c < edgeDistance + 1;c++) {
			int index = (j + k) * multiplyWidth + (i + c);
			if (checkIndicator == 1) {
				if (k != 0 && c != 0 && inBuffer[index] == 255) 
					return 1;
			}
			else {
				if (k != 0 && c != 0 && inBuffer[index] == 0)
					return 0;
			}
		}
	}
	if (checkIndicator == 1)
		return 0;
	return 1;

}

void set_input_to_black_and_white(int* inBuffer, int width, int height) {
	for (int i = 0; i < width * height;i++) {

		if (inBuffer[i] > THRESHOLD)
			inBuffer[i] = 255;
		else
			inBuffer[i] = 0;
	}
}

/**
* @brief Serial version of edge detection algorithm
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param width image width
* @param height image height
*/
void filter_serial_edge_detection(int *inBuffer, int *outBuffer, int width, int height)	
{
	set_input_to_black_and_white(inBuffer, width, height);
	// checking suroundings for every pixel and setting output buffer to coresponding color
	short P = 0, O = 0;
	for (int i = edgeDistance;i < width - edgeDistance;i++) {
		for (int j = edgeDistance; j < height - edgeDistance;j++) {
			P = check_around_pixel(i, j, width, inBuffer, 1);
			O = check_around_pixel(i, j, width, inBuffer, 0);
			if (abs(P - O) == 0) {
				outBuffer[(j - 1) * width + i] = 0;
			}
			else {
				outBuffer[(j - 1) * width + i] = 255;
			}
		}
	}
}


void filter_serial_for_parallel_edge_detection(int* inBuffer, int* outBuffer, int startWidth, int startHeight, int width, int height, int multiplyWidth) {
	short P = 0, O = 0;
	
	for (int i = startWidth + edgeDistance - 1;i < width - edgeDistance;i++) {
		for (int j = startHeight + edgeDistance - 1; j < height - edgeDistance;j++) {
			
			P = check_around_pixel(i, j, width, inBuffer, 1, multiplyWidth);
			O = check_around_pixel(i, j, width, inBuffer, 0, multiplyWidth);
			if (abs(P - O) == 0) {
				
				outBuffer[(j - 1) * multiplyWidth + i] = 0;
			}
			else {
				outBuffer[(j - 1) * multiplyWidth + i] = 255;
			}
		}
	}
}


/**
* @brief Parallel version of edge detection algorithm
* 
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param width image width
* @param height image height
*/
void filter_parallel_edge_detection(int *inBuffer, int *outBuffer, int startWidth, int startHeight, int width, int height, int multiplyWidth)
{
	task_group g;
	if (abs(height - startHeight) / 2 < CUTOFF || abs(width - startWidth) / 2 < CUTOFF) {
		filter_serial_for_parallel_edge_detection(inBuffer, outBuffer, startWidth, startHeight, width, height, multiplyWidth);
	}
	else {
		g.run([&] {filter_serial_for_parallel_edge_detection(inBuffer, outBuffer, startWidth, startHeight, startWidth + (width - startWidth) / 2 + edgeDistance, startHeight + (height - startHeight) / 2 + edgeDistance, multiplyWidth); });
		g.run([&] {filter_serial_for_parallel_edge_detection(inBuffer, outBuffer, startWidth + (width - startWidth) / 2 - edgeDistance, startHeight, width, startHeight + (height - startHeight) / 2 + edgeDistance, multiplyWidth); });
		g.run([&] {filter_serial_for_parallel_edge_detection(inBuffer, outBuffer, startWidth, startHeight + (height - startHeight) / 2 - edgeDistance, startWidth + (width - startWidth) / 2 + edgeDistance, height, multiplyWidth); });
		g.run([&] {filter_serial_for_parallel_edge_detection(inBuffer, outBuffer, startWidth + (width - startWidth) / 2 - edgeDistance, startHeight + (height - startHeight) / 2 - edgeDistance, width, height, multiplyWidth); });
		g.wait();
	}
}

/**
* @brief Function for running test.
*
* @param testNr test identification, 1: for serial version, 2: for parallel version
* @param ioFile input/output file, firstly it's holding buffer from input image and than to hold filtered data
* @param outFileName output file name
* @param outBuffer buffer of output image
* @param width image width
* @param height image height
*/


void run_test_nr(int testNr, BitmapRawConverter* ioFile, char* outFileName, int* outBuffer, unsigned int width, unsigned int height)
{

	auto buffer = ioFile->getBuffer();


	cout << "Starting time measurement of edge detection" << endl;
	auto startTime = steady_clock::now();

	switch (testNr)
	{
		case 1:
			cout << "Running serial version of edge detection using Prewitt operator" << endl;
			filter_serial_prewitt(buffer, outBuffer, width, height, 1, 1, width);
			break;
		case 2:
			cout << "Running parallel version of edge detection using Prewitt operator" << endl;
			filter_parallel_prewitt(buffer, outBuffer, 1, 1, width, height, width);
			break;
		case 3:
			cout << "Running serial version of edge detection" << endl;
			filter_serial_edge_detection(buffer, outBuffer, width, height);
			break;
		case 4:
			cout << "Running parallel version of edge detection" << endl;
			set_input_to_black_and_white(buffer, width, height);
			filter_parallel_edge_detection(buffer, outBuffer, 1, 1, width, height, width);
			break;
		default:
			cout << "ERROR: invalid test case, must be 1, 2, 3 or 4!";
			break;
	}
	auto endTime = steady_clock::now();

	int testDuration = duration_cast<milliseconds>(endTime - startTime).count();
	cout << "Total Time elapsed = " << testDuration << " ms" << endl;

	ioFile->setBuffer(outBuffer);
	ioFile->pixelsToBitmap(outFileName);
}

/**
* @brief Print program usage.
*/
void usage()
{
	cout << "\n\ERROR: ca ll program like: " << endl << endl; 
	cout << "ProjekatPP.exe";
	cout << " input.bmp";
	cout << " outputSerialPrewitt.bmp";
	cout << " outputParallelPrewitt.bmp";
	cout << " outputSerialEdge.bmp";
	cout << " outputParallelEdge.bmp" << endl << endl;
}

int main(int argc, char * argv[])
{

	if(argc != __ARG_NUM__)
	{
		usage();
		return 0;
	}
	setParameters();

	BitmapRawConverter inputFile(argv[1]);
	BitmapRawConverter outputFileSerialPrewitt(argv[1]);
	BitmapRawConverter outputFileParallelPrewitt(argv[1]);
	BitmapRawConverter outputFileSerialEdge(argv[1]);
	BitmapRawConverter outputFileParallelEdge(argv[1]);

	unsigned int width, height;

	int test;
	
	width = inputFile.getWidth();
	height = inputFile.getHeight();

	int* outBufferSerialPrewitt = new int[width * height];
	int* outBufferParallelPrewitt = new int[width * height];

	memset(outBufferSerialPrewitt, 0x0, width * height * sizeof(int));
	memset(outBufferParallelPrewitt, 0x0, width * height * sizeof(int));

	int* outBufferSerialEdge = new int[width * height];
	int* outBufferParallelEdge = new int[width * height];

	memset(outBufferSerialEdge, 0x0, width * height * sizeof(int));
	memset(outBufferParallelEdge, 0x0, width * height * sizeof(int));

	// serial version Prewitt
	run_test_nr(1, &outputFileSerialPrewitt, argv[2], outBufferSerialPrewitt, width, height);

	// parallel version Prewitt
	run_test_nr(2, &outputFileParallelPrewitt, argv[3], outBufferParallelPrewitt, width, height);

	// serial version special
	run_test_nr(3, &outputFileSerialEdge, argv[4], outBufferSerialEdge, width, height);

	// parallel version special
	run_test_nr(4, &outputFileParallelEdge, argv[5], outBufferParallelEdge, width, height);

	// verification
	cout << "Verification: ";
	test = memcmp(outBufferSerialPrewitt, outBufferParallelPrewitt, width * height * sizeof(int));

	if(test != 0)
	{
		cout << "Prewitt FAIL!" << endl;
	}
	else
	{
		cout << "Prewitt PASS." << endl;
	}

	test = memcmp(outBufferSerialEdge, outBufferParallelEdge, width * height * sizeof(int));

	if(test != 0)
	{
		cout << "Edge detection FAIL!" << endl;
	}
	else
	{
		cout << "Edge detection PASS." << endl;
	}

	delete [] outBufferSerialPrewitt;
	delete [] outBufferParallelPrewitt;

	delete [] outBufferSerialEdge;
	delete [] outBufferParallelEdge;

	delete[] filterHor;
	delete[] filterVer;

	return 0;
} 