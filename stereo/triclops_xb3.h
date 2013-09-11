#include <iostream>
#include <triclops.h>
#include <assert.h>
#include <math.h>
#include "bumblebee_xb3.h"
#include "lodepng.h"

#pragma once

void flycap2triclops(const Image& one, const Image& two, TriclopsInput* merged) {
  // NEVER TESTED/DEBUGED BECAREFUL!!!!  
  Image mono_one, mono_two;
  one.Convert( PIXEL_FORMAT_MONO8, &mono_one );
  two.Convert( PIXEL_FORMAT_MONO8, &mono_two );

  if (mono_one.GetCols() != mono_two.GetCols() or mono_one.GetRows() != mono_two.GetRows() or
      mono_one.GetDataSize() != mono_two.GetDataSize()) {
    printf("Error: Image sizes must match");
    exit(-1);
  }

  // free previous type
  // if (merged->u.rgb.red) delete merged->u.rgb.red;
  // if (merged->u.rgb.green) delete merged->u.rgb.green;
  // if (merged->u.rgb.blue) delete merged->u.rgb.blue;
  unsigned int dataSize = mono_one.GetDataSize();
  
  // create type
  merged->inputType 	= TriInp_RGB;
  merged->nrows		= mono_one.GetRows();
  merged->ncols		= mono_one.GetCols();
  merged->rowinc	= mono_one.GetCols();
  merged->u.rgb.red   	= new unsigned char[dataSize];
  merged->u.rgb.green 	= new unsigned char[dataSize];
  merged->u.rgb.blue  	= merged->u.rgb.green;

  // copy data over
  std::copy(mono_one.GetData(), mono_one.GetData()+mono_one.GetDataSize(), (unsigned char*)merged->u.rgb.red);
  std::copy(mono_two.GetData(), mono_two.GetData()+mono_two.GetDataSize(), (unsigned char*)merged->u.rgb.green);  
}


TriclopsInput nullTriclopsInput() {
  TriclopsInput input;
  input.u.rgb.red = NULL;
  input.u.rgb.green = NULL;
  input.u.rgb.blue = NULL;
  input.u.rgb32BitPacked.data=NULL;
  return input;
}

void rgb2bgru(const std::vector<unsigned char>& rgb, unsigned char** bgru) {
  // This allocates the data!
  assert((rgb.size() % 3) == 0);
  if (*bgru) delete *bgru;
  *bgru = new unsigned char[(rgb.size()/3)*4];
  int pixel = 0;
  for (int i = 0; i < (rgb.size()/3); i++) {
    for (int ii = 2; ii >= 0; ii--) {
      (*bgru)[ii + i*4] = rgb[pixel++];
    }
  }
}

char* bgr2bgru(char* bgr, int width, int height) {
  char* bgru = new char[width*height*4];
  int pixel = 0;
  for (int i = 0; i < width*height*4; i++)
    if (i%4 < 3) bgru[i] = bgr[pixel++];
  return bgru;
}

void delaceRGB(const std::vector<unsigned char>& rgb, unsigned char** red, unsigned char** green, unsigned char** blue) {
  // This allocates the data!
  assert((rgb.size() % 3) == 0);
  int npixels = rgb.size()/3;

  if (*red) delete *red;
  if (*green) delete *green;
  if (*blue) delete *blue;
  
  *red = new unsigned char[npixels];
  *green = new unsigned char[npixels];
  *blue = new unsigned char[npixels];

  for (int i = 0; i < rgb.size(); i++) {
    if ((i % 3) == 0)
      (*red)[i/3] = rgb[i];
    else if (((i+1)%3) == 0)
      (*green)[i/3] = rgb[i];      
    else if (((i+2)%3) == 0)
      (*blue)[i/3] = rgb[i];
  }
}


void interlaceRGB(const int& nrows, const int& ncols, const int& rowinc,
		  const unsigned char* red, const unsigned char* green,
		  const unsigned char* blue, std::vector<unsigned char>* rgb) {

  rgb->resize(nrows*ncols*3);
  std::vector<const unsigned char*> zs(3);
  zs[0] = red; zs[1] = green; zs[2] = blue;

  for (int r = 0; r < nrows; r++) {
    for (int c = 0; c < ncols; c++) {
      for (int z = 0; z < 3; z++) {
	(*rgb)[z+c*3+r*ncols*3] = zs[z][c + r*rowinc];
      }
    }
  }
}
		  

double sRGB_to_linear(double x) {
  if (x < 0.04045) return x/12.92;
  return pow((x+0.055)/1.055,2.4);
}

double linear_to_sRGB(double y) {
  if (y <= 0.0031308) return 12.92 * y;
  return 1.055 * pow(y, 1/2.4) - 0.055;
}

unsigned char grayPixel(unsigned char R, unsigned char G, unsigned char B) {
  double R_linear = sRGB_to_linear(R/255.0);
  double G_linear = sRGB_to_linear(G/255.0);
  double B_linear = sRGB_to_linear(B/255.0);
  double gray_linear = 0.299 * R_linear + 0.587 * G_linear + 0.114 * B_linear;
  return round(linear_to_sRGB(gray_linear) * 255);
}

void rgb2gray(const unsigned char* rgb, const unsigned int& rgbSize, unsigned char** gray,
	      bool RGBU=false) {
  assert((rgbSize % 3) == 0);
  if (*gray) delete *gray;
  *gray = new unsigned char[rgbSize/3];
  unsigned int pixel = 0;
  unsigned int skipSize = (RGBU) ? 4 : 3;
  for (int i = 0; i < rgbSize; i+=skipSize ){
    (*gray)[pixel++] = grayPixel(rgb[i], rgb[i+1], rgb[i+2]);
  }
}

void rgb2gray(const std::vector<unsigned char>& rgb, unsigned char** gray, bool RGBU=false) {
  rgb2gray(rgb.data(), rgb.size(), gray, RGBU);
}

void rgb2stereo(const TriclopsInput& left, const TriclopsInput& right,
		TriclopsInput* stereo) {

  assert( left.nrows == right.nrows );
  assert( left.ncols == right.ncols );
  assert( left.rowinc == 4 * left.ncols );
  assert( right.rowinc == 4 * right.ncols );
  assert( left.inputType == TriInp_RGB_32BIT_PACKED );
  assert( right.inputType == TriInp_RGB_32BIT_PACKED );  
  
  stereo->inputType = TriInp_RGB;
  stereo->nrows     = left.nrows;
  stereo->ncols     = left.ncols;
  stereo->rowinc    = stereo->ncols;

  rgb2gray((unsigned char*) right.u.rgb32BitPacked.data, right.nrows*right.ncols*4,
	   (unsigned char**) &stereo->u.rgb.red, true);
  rgb2gray((unsigned char*) left.u.rgb32BitPacked.data, left.nrows*left.ncols*4,
	   (unsigned char**) &stereo->u.rgb.green, true);
  stereo->u.rgb.blue = stereo->u.rgb.green;
}

void pngReadToTriclopsInput( std::string filename,
			     TriclopsInput* input ) {

  std::vector<unsigned  char> image;
  unsigned width, height;
  lodepng::decode(image, width, height, filename, LCT_RGB);

  input->inputType = TriInp_RGB_32BIT_PACKED;
  input->nrows     = height;
  input->ncols     = width;
  input->rowinc    = 4 * width;
  rgb2bgru(image, (unsigned char**)&input->u.rgb32BitPacked.data);
}

void pngReadToTriclopsInputRGB( std::string filename,
				TriclopsInput* input) {

  std::vector<unsigned  char> image;
  unsigned width, height;
  lodepng::decode(image, width, height, filename, LCT_RGB);

  input->inputType = TriInp_RGB;
  input->nrows     = height;
  input->ncols     = width;
  input->rowinc    = width;
  delaceRGB(image, (unsigned char**) &input->u.rgb.red,
	    (unsigned char**) &input->u.rgb.green, (unsigned char **) &input->u.rgb.blue);
}


void pngReadToStereoTriclopsInputRGB( const std::string& leftFilename,
				      const std::string& rightFilename,
				      TriclopsInput* input) {
  std::vector<unsigned  char> leftImage;
  unsigned leftWidth, leftHeight;
  lodepng::decode(leftImage, leftWidth, leftHeight, leftFilename, LCT_RGB);

  std::vector<unsigned  char> rightImage;
  unsigned rightWidth, rightHeight;
  lodepng::decode(rightImage, rightWidth, rightHeight, rightFilename, LCT_RGB);
  
  input->inputType = TriInp_RGB;
  input->nrows     = leftHeight;
  input->ncols     = leftWidth;
  input->rowinc    = leftWidth;
  rgb2gray(leftImage, (unsigned char**) &input->u.rgb.red);
  rgb2gray(rightImage, (unsigned char**) &input->u.rgb.green);
  input->u.rgb.blue = input->u.rgb.green;
}

void pngReadToStereoTriclopsInputRGB( const std::string& leftFilename,
				      const std::string& centerFilename,
				      const std::string& rightFilename,
				      TriclopsInput* input) {
  
  pngReadToStereoTriclopsInputRGB(leftFilename, centerFilename, input);
  std::vector<unsigned  char> rightImage;
  unsigned rightWidth, rightHeight;
  lodepng::decode(rightImage, rightWidth, rightHeight, rightFilename, LCT_RGB);
  input->u.rgb.green=NULL;
  rgb2gray(rightImage, (unsigned char**) &input->u.rgb.green);
}


void pngWriteFromTriclopsColorImage( const std::string& filename,
				     const TriclopsColorImage& image ) {
  std::vector<unsigned char> rgb;
  interlaceRGB(image.nrows, image.ncols, image.rowinc,
  	       image.red, image.green, image.blue, &rgb);
  // interlaceRGB(image.nrows, image.ncols, image.rowinc,
  // 	       image.blue, image.green, image., &rgb);  
  lodepng::encode(filename, rgb, image.ncols, image.nrows, LCT_RGB);
}

void pngWriteFromTriclopsImage( const std::string& filename,
				const TriclopsImage& image ) {
  assert(image.rowinc == image.ncols);
  lodepng::encode(filename, (unsigned char*)image.data, image.ncols, image.nrows, LCT_GREY);
}

void pngWriteFromTriclopsImage16( const std::string& filename,
				  const TriclopsImage16& image ) {
  // Not sure if this actually works...
  assert( image.rowinc == 2*image.ncols );

  std::vector<unsigned char> data((unsigned char*) image.data,
				  (unsigned char*) (image.data+(image.nrows*image.ncols)));
  lodepng::encode(filename, data.data(),
		  image.ncols, image.nrows, LCT_GREY, 16);
}

void disparity2ptsfile(const std::string& filename,
		       const TriclopsContext& context,
		       const TriclopsColorImage& colorImage,
		       const TriclopsImage16& disparityImage ) {
  
  FILE*	       pointFile;  
  float	               x, y, z; 
  int	               r, g, b;
  int		       nPoints = 0;
  int		       pixelinc ;
  int		       i, j, k;
  unsigned short*      row;
  unsigned short       disparity;

  pointFile = fopen( filename.c_str(), "w+" );  

  // Determine the number of pixels spacing per row
  pixelinc = disparityImage.rowinc/2;
  for ( i = 0, k = 0; i < disparityImage.nrows; i++ )
    {
      row     = disparityImage.data + i * pixelinc;
      for ( j = 0; j < disparityImage.ncols; j++, k++ )
	{
	  disparity = row[j];
	 
	  // do not save invalid points
	  if ( disparity < 0xFF00 )
	    {
	      // convert the 16 bit disparity value to floating point x,y,z
	      triclopsRCD16ToXYZ( context, i, j, disparity, &x, &y, &z );
	      
	      // look at points within a range
	      if ( z < 5.0 ) {
		r = (int)colorImage.red[k];
		g = (int)colorImage.green[k];
		b = (int)colorImage.blue[k];		  
		fprintf( pointFile, "%f %f %f %d %d %d %d %d\n", x, y, z, r, g, b, i, j );
		nPoints++;
	      }
	    }
	}
    }
}




// TriclopsBool
// freeImage( TriclopsImage* pimage )
// {
//    if( pimage->data != NULL )
//    {
//       free( pimage->data );
//       pimage->data = NULL;
//    }

//    return true;
// }


// TriclopsBool
// freeImage16( TriclopsImage16* pimage )
// {
//    if( pimage->data != NULL )
//    {
//       free( pimage->data );
//       pimage->data = NULL;
//    }

//    return true;
// }

TriclopsBool
freeColorImage( TriclopsColorImage* pimage )
{
  if ( pimage->red != NULL )
    {
      free( pimage->red );
      pimage->red = NULL;
    }
  if ( pimage->green != NULL )
    {
      free( pimage->green );
      pimage->green = NULL;
    }
  if ( pimage->blue != NULL )
    {
      free( pimage->blue );
      pimage->blue = NULL;
    }
  return true;
}
