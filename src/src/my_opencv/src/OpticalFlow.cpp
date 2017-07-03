#include <cv.h>
#include <highgui.h>

int
main (int argc, char **argv)
{
  int i, j, dx, dy, rows, cols;
  IplImage *src_img1, *src_img2, *dst_img1, *dst_img2;
  CvMat *velx, *vely;
  CvTermCriteria criteria;

  if (argc != 3 ||
      (src_img1 = cvLoadImage (argv[1], CV_LOAD_IMAGE_GRAYSCALE)) == 0 ||
      (src_img2 = cvLoadImage (argv[2], CV_LOAD_IMAGE_GRAYSCALE)) == 0)
    return -1;

  dst_img1 = cvLoadImage (argv[2], CV_LOAD_IMAGE_COLOR);
  dst_img2 = (IplImage *) cvClone (dst_img1);

  // (1)速度ベクトルを格納する構造体の確保，等
  cols = src_img1->width;
  rows = src_img1->height;
  velx = cvCreateMat (rows, cols, CV_32FC1);
  vely = cvCreateMat (rows, cols, CV_32FC1);
  cvSetZero (velx);
  cvSetZero (vely);
  criteria = cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 64, 0.01);

  // (2)オプティカルフローを計算（HS）
  cvCalcOpticalFlowHS (src_img1, src_img2, 0, velx, vely, 100.0, criteria);
  // (3)オプティカルフローを描画（HS）
  for (i = 0; i < cols; i += 5) {
    for (j = 0; j < rows; j += 5) {
      dx = (int) cvGetReal2D (velx, j, i);
      dy = (int) cvGetReal2D (vely, j, i);
      cvLine (dst_img1, cvPoint (i, j), cvPoint (i + dx, j + dy), CV_RGB (255, 0, 0), 1, CV_AA, 0);
    }
  }

  // (4)オプティカルフローを計算（LK）
  cvCalcOpticalFlowLK (src_img1, src_img2, cvSize (15, 15), velx, vely);
  // (5)計算されたフローを描画（LK）
  for (i = 0; i < cols; i += 5) {
    for (j = 0; j < rows; j += 5) {
      dx = (int) cvGetReal2D (velx, j, i);
      dy = (int) cvGetReal2D (vely, j, i);
      cvLine (dst_img2, cvPoint (i, j), cvPoint (i + dx, j + dy), CV_RGB (255, 0, 0), 1, CV_AA, 0);
    }
  }

  // (6)オプティカルフローの表示
  cvNamedWindow ("ImageHS", 1);
  cvShowImage ("ImageHS", dst_img1);
  cvNamedWindow ("ImageLK", 1);
  cvShowImage ("ImageLK", dst_img2);
  cvWaitKey (0);

  cvDestroyWindow ("ImageHS");
  cvDestroyWindow ("ImageLK");
  cvReleaseImage (&src_img1);
  cvReleaseImage (&src_img2);
  cvReleaseImage (&dst_img1);
  cvReleaseImage (&dst_img2);
  cvReleaseMat (&velx);
  cvReleaseMat (&vely);

  return 0;
}
