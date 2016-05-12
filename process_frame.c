/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#define IMG_SIZE NUM_COLORS*OSC_CAM_MAX_IMAGE_WIDTH*OSC_CAM_MAX_IMAGE_HEIGHT
#define DEBUG_LVL   0
/*
 *   CONSTANTS
 */
const int nc = OSC_CAM_MAX_IMAGE_WIDTH;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT;

//const float avgFac = 0.95;

/* skip pixel at border */
const int Border = 2;

/* after this number of steps object is set to background */
//const int frgLimit = 100;

/* minimum size of objects (sum of all pixels) */
const int MinArea = 500*3;
char txt_testat[] = "Testat 2 Pascal Haefliger, Fabian Niederberger";

/*
 * VARIABLES
 */
float bgrImg[IMG_SIZE];

struct OSC_VIS_REGIONS ImgRegions;/* these contain the foreground objects */

int16 imgDx[IMG_SIZE];
int16 imgDy[IMG_SIZE];

/*
 *  Prototypes
 */
void Derivative();
//void SetBackground();
void Erode_3x3(int InIndex, int OutIndex);
void Dilate_3x3(int InIndex, int OutIndex);
void DetectRegions();
void DrawBoundingBoxes();
void ProcessRegions(void);


void ResetProcess()
{
    //SetBackground();
}


void ProcessFrame()
{
    //initialize counters
    if(data.ipc.state.nStepCounter == 1)
    {
        //SetBackground();
#if DEBUG_LVL >= 1
        printf("App started\n");
#endif
    }
    else
    {
        Derivative();

        // Disable because of performance issue
        // We don't need an opening on the dI, because we only need Dx and Dy
        //Erode_3x3(THRESHOLD, INDEX0);
        //Dilate_3x3(INDEX0, THRESHOLD);  // Opening, remove noise

        DetectRegions();        // Locate important Regions
        DrawBoundingBoxes();

        ProcessRegions();

        DrawString(0, 0, strlen(txt_testat), LARGE, CYAN, txt_testat);
    }
}

void Derivative()
{
    int r, c;
    //set result buffer to zero
    memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);

    //loop over the rows (heigth)
    for(r = Border*nc; r < (nr-Border)*nc; r += nc)
    {
        //loop over the columns (width)
        for(c = Border; c < (nc-Border); c++)
        {
            unsigned char* p = &data.u8TempImage[SENSORIMG][r+c];

            /* implement Sobel filter in x-direction */
            int32 dx = -(int32) *(p-nc-1) + 0 + (int32) *(p-nc+1)
                       -2* (int32) *(p-1) + 0 + 2* (int32) *(p+1)
                       -(int32) *(p+nc-1) + 0 + (int32) *(p+nc+1);

            int32 dy = -(int32) *(p-nc-1)  -  2* (int32) *(p-nc)  - (int32) *(p-nc+1)
                       +         0         +        0             + 0
                       + (int32) *(p+nc-1) +  2* (int32) *(p+nc)  + (int32) *(p+nc+1);

            /* check if norm is larger than threshold */
            int32 df2 = dx*dx+dy*dy;    // Don't norm (sqrt) the result, because
            int32 thr2 = data.ipc.state.nThreshold*data.ipc.state.nThreshold;
            if(df2 > thr2)  //avoid square root
            {
                //set pixel value to 255 in THRESHOLD image for gui
                data.u8TempImage[THRESHOLD][r+c] = 255;
            }


            //store derivatives (int16 is enough)
            imgDx[r+c] = (int16) dx;
            imgDy[r+c] = (int16) dy;

#if DEBUG_LVL >= 5
            printf("dx is %i\n",dx);
            printf("dy is %i\n",dy);
#endif

            //possibility to visualize data
            data.u8TempImage[BACKGROUND][r+c] = (uint8) MAX(0, MIN(255,128+(dy+dx)/2));

            //Opening();    // Don't make opening here, because of the ammount of content switching
        }
    }
}


void ProcessRegions(void)
{
    char text[8];
    double angle;
    for(uint16_t i = 0; i < ImgRegions.noOfObjects; i++)  // Loop over all boxes
    {

        if(ImgRegions.objects[i].area > MinArea)    // Loop only over big boxes
        {
            uint16_t result[4] = {0,0,0,0};
            memcpy(text, "          ", 8);

            struct OSC_VIS_REGIONS_RUN* curRun = ImgRegions.objects[i].root;

            while(curRun != NULL)
            {
                for (uint16_t curCol = curRun->startColumn; curCol <= curRun->endColumn; curCol++)
                {
                    uint16_t curRow = curRun->row;

                    //AngleBinning
                    angle = atan2(imgDy[curRow * nc + curCol], imgDx[curRow * nc + curCol]);
                    if(angle < 0)
                        angle += M_PI;
                    else if(angle > M_PI)
                        angle -= M_PI;

#if DEBUG >= 4
                    if(angle > M_PI || angle < 0)
                        printf("%f\n",angle);
#endif

                    if ((angle <= (M_PI_4 / 2)) && (angle >= 0))
                        result[0]++;
                    else if ((angle >= (1 * M_PI_4 / 2)) && (angle <= (3 * M_PI_4 / 2)))
                        result[1]++;
                    else if ((angle >= (3 * M_PI_4 / 2)) && (angle <= (5 * M_PI_4 / 2)))
                        result[2]++;
                    else if ((angle >= (5 * M_PI_4 / 2)) && (angle <= (7 * M_PI_4 / 2)))
                        result[3]++;
					else if (angle <= M_PI)
						result[0]++;
					else
						printf("ERROR!!, Binning out of bounds.\n");
                }
                curRun = curRun->next;
            }

            //Calc the most common angle
            uint16_t max = 0;
            for(uint16 x=0; x<4; x++)
            {
#if DEBUG_LVL >= 1
                printf("angle %i has %i elements\n",x, result[x]);
#endif
                if(result[x] > result[max])
                    max = x;
            }

#if DEBUG_LVL >= 1
            if(result[max] == 0)
                printf("Binning failed\n");
#endif

            // Print correct String for each big box
            if(max == 0)
                memcpy(text, "0 deg  ", 8);
            else if(max == 1)
                memcpy(text, "45 deg ", 8);
            else if(max == 2)
                memcpy(text, "90 deg ", 8);
            else if(max == 3)
                memcpy(text, "135 deg", 8);

            DrawString(ImgRegions.objects[i].centroidX,
                       ImgRegions.objects[i].centroidY,
                       7,
                       LARGE,
                       GREEN,
                       text);

        }   //endif Loop only over big boxes
    }   // endif Loop over all boxes
}

/*
void SetBackground()
{
    int r, c;

    //loop over the rows
    for(r = Border*nc; r < (nr-Border)*nc; r += nc)
    {
        //loop over the columns
        for(c = Border; c < (nc-Border); c++)
        {
            bgrImg[r+c] = (float) data.u8TempImage[SENSORIMG][r+c];
        }
    }
    //set all counters to zero
    memset(data.u8TempImage[INDEX1], 0, IMG_SIZE);
}
*/

void Erode_3x3(int InIndex, int OutIndex)
{
    int c, r;

    for(r = Border*nc; r < (nr-Border)*nc; r += nc)
    {
        for(c = Border; c < (nc-Border); c++)
        {
            unsigned char* p = &data.u8TempImage[InIndex][r+c];
            // From OutIndex --> BACKGROUND
            data.u8TempImage[OutIndex][r+c] = *(p-nc-1) & *(p-nc) & *(p-nc+1) &
                                                *(p-1)    & *p      & *(p+1)    &
                                                *(p+nc-1) & *(p+nc) & *(p+nc+1);
        }
    }
}

void Dilate_3x3(int InIndex, int OutIndex)
{
    int c, r;

    for(r = Border*nc; r < (nr-Border)*nc; r += nc)
    {
        for(c = Border; c < (nc-Border); c++)
        {
            unsigned char* p = &data.u8TempImage[InIndex][r+c];
            // From OutIndex --> BACKGROUND
            data.u8TempImage[OutIndex][r+c] = *(p-nc-1) | *(p-nc) | *(p-nc+1) |
                                                *(p-1)    | *p      | *(p+1)    |
                                                *(p+nc-1) | *(p+nc) | *(p+nc+1);
        }
    }
}

void DetectRegions()
{
    struct OSC_PICTURE Pic;

    //set pixel value to 1 in INDEX0 because the image MUST be binary (i.e. values of 0 and 1)
    for(int i = 0; i < IMG_SIZE; i++)
    {
        data.u8TempImage[INDEX0][i] = data.u8TempImage[THRESHOLD][i] ? 1 : 0;
    }

    //wrap image INDEX0 in picture struct
    Pic.data = data.u8TempImage[INDEX0];
    Pic.width = nc;
    Pic.height = nr;
    Pic.type = OSC_PICTURE_BINARY;

    //now do region labeling and feature extraction
    OscVisLabelBinary(&Pic, &ImgRegions);
    OscVisGetRegionProperties(&ImgRegions);
}


void DrawBoundingBoxes()
{
    for(uint16 i = 0; i < ImgRegions.noOfObjects; i++)
    {
        if(ImgRegions.objects[i].area > MinArea)
        {
            DrawBoundingBox(ImgRegions.objects[i].bboxLeft,
                            ImgRegions.objects[i].bboxTop,
                            ImgRegions.objects[i].bboxRight,
                            ImgRegions.objects[i].bboxBottom, FALSE, GREEN);
        }
    }
}
