#include "reidentifier.h"
#include <fstream>
#include <iostream>
#include <QFile>
#include <QTextStream>
#include <ros/ros.h>
#include <ros/package.h>

ReIdentifier::ReIdentifier()
{
}

int sub2ind(int x, int y, int width)
{
    int offset;
    if (y>0)
        offset = (y-1)*width;
    else
        offset = 0;
    return offset+x;

}

void VectorToCvMat(Vector<double> data, cv::Mat *mat)
{
    for (int x=0; x<mat->cols; x++)
    {
        for (int y=0; y<mat->rows; y++)
        {
            int ind_vector = sub2ind(x,y,Globals::dImWidth);
//            cout << "index: " << ind_vector << endl;
            if (ind_vector >= data.getSize() || ind_vector < 0)
            {
                cout << "Error! " << ind_vector << " (" << x << "," << y << ") is out of range (0," << data.getSize()  << ")" << endl;
            }
            else
            {
                mat->at<double>(y,x) = data[ind_vector];
            }
        }
    }

}

void VectorToCvMatandThreshold(const Vector<double> &data, cv::Mat *mat, float thrLower, float thrUpper)
{
    for (int x=0; x<mat->cols; x++)
    {
        for (int y=0; y<mat->rows; y++)
        {
            int ind_vector = sub2ind(x,y,Globals::dImWidth);
//            cout << "index: " << ind_vector << endl;
            if (ind_vector >= data.getSize() || ind_vector < 0)
            {
                cout << "Error! " << ind_vector << " (" << x << "," << y << ") is out of range (0," << data.getSize()  << ")" << endl;
            }
            else
            {
//                cout << "(" << x << "," << y << ")" << endl;
                if ((data[ind_vector]> thrLower) && (data[ind_vector]< thrUpper))
                {
                    mat->at<float>(y,x) = 1;

                }
                else
                {
                    mat->at<float>(y,x) = 0;
                }
            }
        }
    }

}

void PrintMatrix(cv::Mat Mat)
{
    float valFloat;
    unsigned char valUChar;
    int type = Mat.type();

    stringstream vals;
    for (int i = 0;i<Mat.rows;i++)
    {
        vals.str("");
        for(int j = 0;j<Mat.cols;j++)
        {
            valUChar = Mat.at<unsigned char>(i,j);
            valFloat = Mat.at<float>(i, j);
            if (type == CV_32FC1)
            {
//                std::cout << valFloat << "|";
                vals << valFloat << "|";

            }
            if (type == CV_8UC1)
            {
//                std::cout << static_cast<unsigned>(valUChar) << "|";
                vals << static_cast<unsigned>(valUChar) << "|";
            }

        }


//        std::cout << "|" << std::endl;
        vals << "|" << std::endl;
        ROS_INFO("%s",vals.str().c_str());
    }

}

void writeTXTasCSV(std::string filename, cv::Mat data)
{
    float valFloat;
    unsigned char valUChar;
    int type = data.type();

    ofstream file;
    file.open(filename.c_str());
//    QFile *file = new QFile(filename);
//    file->open(QIODevice::WriteOnly | QIODevice::Text);
//    QTextStream outs(file);

    if (data.rows == 1)
    {

        for (int i=0;i<data.cols;i++)
        {
            valUChar = data.at<unsigned char>(i);
            valFloat = data.at<float>(i);
            if (type == CV_32FC1)
            {
//                outs << valFloat << ", ";
                file << valFloat << ", ";
            }
            if (type == CV_8UC1)
            {
//                outs << static_cast<unsigned>(valUChar) << ", ";
                file << static_cast<unsigned>(valUChar) << ", ";
            }
        }
    }
    else if (data.cols == 1)
    {
        for (int i=0;i<data.rows;i++)
        {
            valUChar = data.at<unsigned char>(i);
            valFloat = data.at<float>(i);
            if (type == CV_32FC1)
            {
//                outs << valFloat << ", ";
                file << valFloat << ", ";
            }
            if (type == CV_8UC1)
            {
//                outs << static_cast<unsigned>(valUChar) << ", ";
                file << static_cast<unsigned>(valUChar) << ", ";
            }
        }
    }
    else
    {
        for (int i=0;i<data.rows;i++)
        {
            for (int j=0; j< data.cols; j++)
            {
                valUChar = data.at<unsigned char>(i,j);
                valFloat = data.at<float>(i,j);
                if (type == CV_32FC1)
                {
//                    outs << valFloat << ", ";
                    file << valFloat << ", ";
                }
                if (type == CV_8UC1)
                {
//                    outs << static_cast<unsigned>(valUChar) << ", ";
                    file << static_cast<unsigned>(valUChar) << ", ";
                }
            }
//            outs << endl;
            file << endl;
        }
    }


    file.close();
}

void sumOverDim(cv::Mat *data, cv::Mat *sum, int dim)
{
    if (dim == 0)
    {
        // Sum over columns, output is a single row
        for (int i=0; i < data->rows; i++)
        {
            float sumCols=0;
            for (int j=0; j < data->cols; j++)
            {
                sumCols += data->at<float>(i,j);
            }
            sum->at<float>(i) = sumCols;
        }
    }
    else
    {
        // Sum over rows, output is a single column
        for (int j=0; j < data->cols; j++)
        {
            float sumRows=0;
            for (int i=0; i < data->rows; i++)
            {
                sumRows += data->at<float>(i,j);
            }
            sum->at<float>(j) = sumRows;
        }
    }
}

void calcMeanandStdDev(vector<float> data, vector<float> *stat)
{
    float sum=0;
//    stringstream data_str;
    for (int i=0;i<data.size();i++)
    {
        sum+=data[i];
//        data_str << data[i] << ", ";
    }
//    ROS_INFO("vector: %s", data_str.str().c_str());
    float avg = sum/data.size();
    stat->push_back(avg);
    sum=0;
    for (int i=0;i<data.size();i++)
    {
        sum+=(data[i] - avg) * (data[i] - avg);
    }

    float var = sum/(data.size()-1);
    stat->push_back(sqrt(var));

}


void calcHistogram(cv::Mat data, cv::Mat *data_hist, int nBins)
{
//    float minVal=std::numeric_limits<float>::max();
    float maxVal=std::numeric_limits<float>::min();
    for (int id=0;id<data.rows;id++)
    {

//        minVal = qMin(minVal,data.at<float>(id));
        maxVal = max(maxVal,data.at<float>(id));

    }
    float minVal = 0;

//    cout << minVal << "," << maxVal << endl;
    float range = maxVal-minVal;
    double step = ((float)range / (float) nBins);

    float binVals[nBins+1];

    float tmp = minVal + (step/2);
    for(int binId=0; binId<nBins+1;binId++)
    {
        binVals[binId]=tmp;
        tmp = tmp + step;

        data_hist->at<float>(binId,0)=0;
        data_hist->at<float>(binId,1)=binVals[binId];

        for (int Id=0;Id<data.rows;Id++)
        {
            float val = data.at<float>(Id);
            if (val>=(binVals[binId]-(step/2)) && val<(binVals[binId]+(step/2)))
            {

                data_hist->at<float>(binId,0)++;

            }


        }

        data_hist->at<float>(binId,0) = data_hist->at<float>(binId,0) / data.rows;
        //qDebug() << data_hist[binId];

    }

}

void ReIdentifier::ConnectedComponentAnalysis(cv::Mat binaryImg, vector<cv::Mat> *blobs, int xOffset)
{
    int height = binaryImg.rows;
    int width = binaryImg.cols;
    /*vector<cv::Mat> blobs*/;

    // Using labels from 2+ for each blob

    int label_count = 2; // starts at 2 because 0, 1 are used already

//    cv::Mat mask(height + 2, width + 2, CV_8UC1);
//    cv::Mat mask(5, 5, CV_8UC1);
    cv::Mat binaryImg_filled = binaryImg;
    binaryImg.convertTo(binaryImg_filled,CV_32FC1);
//    writeTXTasCSV("./data/binaryImg_org.txt",binaryImg);
//    writeTXTasCSV("./data/binaryImg.txt",binaryImg_filled);
    for (int y=0;y<height;y++)
    {

        for (int x=0;x<width;x++)
        {
            if (binaryImg_filled.at<float>(y, x) == 1)
            {

                cv::Rect rect;
                int filled = cv::floodFill(binaryImg_filled,cv::Point(x, y), cv::Scalar(label_count),&rect);

//                cout << label_count << " - " << "(" << x << "," << y << ") - filled: " << filled << endl;

                if (filled > (height * width * 0.05))
                {
                    cv::Mat blob(filled,2,CV_32FC1);
                    blob.setTo(0);

                    int idd=0;
                    for (int i=rect.y;i<rect.y + rect.height;i++)
                    {
                        for (int j=rect.x;j<rect.x + rect.width;j++)
                        {
//                            cout << "(" << j << "," << i << "): " << binaryImg.at<float>(i, j) << endl;
                            if (binaryImg_filled.at<float>(i, j) == label_count)
                            {
//                                cout << "Found label: " << label_count << endl;
                                blob.at<float>(idd,0) = j + xOffset;
                                blob.at<float>(idd,1) = i;
                                idd++;
                            }

                        }
                    }
//                    writeTXTasCSV("./data/blob_" + QString::number(label_count) + ".txt",blob);
                    blobs->push_back(blob);

                }
                label_count++;

            }
        }
    }

//    writeTXTasCSV("./data/binaryImg_filled.txt",binaryImg_filled);

}

void ReIdentifier::SetGPLevel(float gp_y)
{
    gpLevel=gp_y;
}

void ReIdentifier::SetThreshold(float thrs)
{
//    cout << thrs << endl;
    ridentify_thrs=thrs;
}

void ReIdentifier::SetIDs(Vector<int>& IDs)
{
    for (int i=0;i<IDs.getSize();i++)
    {
        matchedIDs.push_back(IDs[i]);
    }
}

float distanceToLine(float x1, float y1, float x2, float y2, float x0, float y0)
{
    float numer = abs((y2-y1)*x0 - (x2-x1)*y0 +x2*y1 - y2*x1);
    float denom = sqrt((y2-y1)*(y2-y1) + (x2-x1)*(x2-x1));

    float distance = numer / denom;

    return distance;

}

void calculateVolume(PointCloudData pcloud,cv::Mat blob, float shoulderLeft_x, float shoulderRight_x, float headPt_y, float neckPt_y, float bbox_y_max, vector<float> &volumes)
{

    float shoulderLeft_x3D = pcloud.X[sub2ind(shoulderLeft_x,bbox_y_max,Globals::dImWidth)];
    float shoulderLeft_z3D = pcloud.Z[sub2ind(shoulderLeft_x,bbox_y_max,Globals::dImWidth)];
    float shoulderRight_x3D = pcloud.X[sub2ind(shoulderRight_x,bbox_y_max,Globals::dImWidth)];
    float shoulderRight_z3D = pcloud.Z[sub2ind(shoulderRight_x,bbox_y_max,Globals::dImWidth)];

    float volFace=0;
    float volBreast=0;
    float volBelly=0;
    for (int i=0;i<blob.rows;i++)
    {
        int x = blob.at<float>(i,0);
        int y = blob.at<float>(i,1);
        float x3D = pcloud.X[sub2ind(x,y,Globals::dImWidth)];
        float z3D = pcloud.Z[sub2ind(x,y,Globals::dImWidth)];
        if (y>headPt_y && y<=neckPt_y)
        {
            volFace+=distanceToLine(shoulderLeft_x3D,shoulderLeft_z3D,shoulderRight_x3D,shoulderRight_z3D,x3D,z3D);
        }
        else if (y>neckPt_y && y<= bbox_y_max)
        {
            volBreast+=distanceToLine(shoulderLeft_x3D,shoulderLeft_z3D,shoulderRight_x3D,shoulderRight_z3D,x3D,z3D);
        }
        else if (y>bbox_y_max && y<= bbox_y_max + (bbox_y_max-neckPt_y))
        {
            volBelly+=distanceToLine(shoulderLeft_x3D,shoulderLeft_z3D,shoulderRight_x3D,shoulderRight_z3D,x3D,z3D);
        }


    }

    volumes.push_back(volFace);
    volumes.push_back(volBreast);
    volumes.push_back(volBelly);



}

void ReIdentifier::calculateVolume(PointCloudData pcloud, cv::Mat blob, float shoulderLeft_x, float shoulderRight_x, float headPt_y, float neckPt_y, float breastPt_y, float bellyPt_y, vector<float> &volumes)
{
    int shoulderMidPoint = (int)((shoulderLeft_x+shoulderRight_x)/2);

    float shoulderLeft_x3D = pcloud.X[sub2ind(shoulderLeft_x+10,breastPt_y,Globals::dImWidth)];
    float shoulderLeft_y3D = pcloud.Y[sub2ind(shoulderLeft_x+10,breastPt_y,Globals::dImWidth)];
    float shoulderLeft_z3D = pcloud.Z[sub2ind(shoulderLeft_x+10,breastPt_y,Globals::dImWidth)];
    float shoulderRight_x3D = pcloud.X[sub2ind(shoulderRight_x-10,breastPt_y,Globals::dImWidth)];
    float shoulderRight_y3D = pcloud.Y[sub2ind(shoulderRight_x-10,breastPt_y,Globals::dImWidth)];
    float shoulderRight_z3D = pcloud.Z[sub2ind(shoulderRight_x-10,breastPt_y,Globals::dImWidth)];

    float maxPersonDepth = 0;
    if (shoulderLeft_z3D>=shoulderRight_z3D)
    {
        maxPersonDepth=shoulderLeft_z3D;
    }
    else
    {
        maxPersonDepth=shoulderRight_z3D;
    }


    vector<float> shoulderLeft3D, shoulderRight3D;
    shoulderLeft3D.push_back(shoulderLeft_x3D);
    shoulderLeft3D.push_back(shoulderLeft_y3D);
    shoulderLeft3D.push_back(shoulderLeft_z3D);
    shoulderRight3D.push_back(shoulderRight_x3D);
    shoulderRight3D.push_back(shoulderRight_y3D);
    shoulderRight3D.push_back(shoulderRight_z3D);
    vector<vector<float> > shoulder3DPts;
    shoulder3DPts.push_back(shoulderLeft3D);
    shoulder3DPts.push_back(shoulderRight3D);
    shoulder3DPtsPeople.push_back(shoulder3DPts);

    float volFace=0;
    float volBreast=0;
    float volBelly=0;

//    string pathFeaturesStr = Globals::bounding_box_path;
//    int pos1 = pathFeaturesStr.find_last_of("/");
//    pathFeaturesStr.erase(pos1+1,41);

//    QFile *file = new QFile( QString(pathFeaturesStr.c_str()) + "/bellyPoints/bellyPoints_" + QString::number(fNo) + ".txt");
//    file->open(QIODevice::WriteOnly | QIODevice::Text);
//    QTextStream outs(file);

//    QFile *file_2 = new QFile( QString(pathFeaturesStr.c_str()) + "/bellyPoints/bellyPoints_YZ_" + QString::number(fNo) + ".txt");
//    file_2->open(QIODevice::WriteOnly | QIODevice::Text);
//    QTextStream out_YZ(file_2);

//    QFile *file_3 = new QFile( QString(pathFeaturesStr.c_str()) + "/bellyPoints/bellyPoints3D_" + QString::number(fNo) + ".txt");
//    file_3->open(QIODevice::WriteOnly | QIODevice::Text);
//    QTextStream out3D(file_3);


//    cv::Mat blob_x = blob.col(0);
//    cv::Mat blob_y = blob.col(1);

//    writeTXTasCSV("./data/blob_x_" + QString::number(fNo) + ".txt",blob_x);
//    writeTXTasCSV("./data/blob_y_" + QString::number(fNo) + ".txt",blob_y);


    int numFaceVolume = 0;
    int numBreastVolume = 0;
    int numBellyVolume = 0;

    for (int i=0;i<blob.rows-1;i++)
    {
        int x1 = blob.at<float>(i,0);
        int y1 = blob.at<float>(i,1);
        int x2 = blob.at<float>(i+1,0);
        int y2 = blob.at<float>(i+1,1);

        if (x1>=shoulderLeft_x+5 && x1<= shoulderRight_x-5)
        {
            float x3D_1 = pcloud.X[sub2ind(x1,y1,Globals::dImWidth)];
            float y3D_1 = pcloud.Y[sub2ind(x1,y1,Globals::dImWidth)];
            float z3D_1 = pcloud.Z[sub2ind(x1,y1,Globals::dImWidth)];
            float x3D_2 = pcloud.X[sub2ind(x2,y2,Globals::dImWidth)];
            float y3D_2 = pcloud.Y[sub2ind(x2,y2,Globals::dImWidth)];
            float z3D_2 = pcloud.Z[sub2ind(x2,y2,Globals::dImWidth)];
            if (z3D_1<=maxPersonDepth+0.1)
            {
                float distance = 0;
                if (shoulderLeft_x3D!=-1 && shoulderLeft_y3D!=-1 && shoulderLeft_z3D!=-1 && shoulderRight_x3D!=-1 && shoulderRight_y3D!=-1 && shoulderRight_z3D!=-1)
                {
                    if (y1>headPt_y && y1<=neckPt_y)
                    {
                        volFace+=(abs(x3D_1-x3D_2))*(abs(y3D_1-y3D_2))*distanceToLine(shoulderLeft_x3D,shoulderLeft_z3D,shoulderRight_x3D,shoulderRight_z3D,x3D_1,z3D_1);
                        numFaceVolume++;
                    }
                    else if (y1>neckPt_y && y1<= breastPt_y)
                    {
                        volBreast+=(abs(x3D_1-x3D_2))*(abs(y3D_1-y3D_2))*distanceToLine(shoulderLeft_x3D,shoulderLeft_z3D,shoulderRight_x3D,shoulderRight_z3D,x3D_1,z3D_1);
                        numBreastVolume++;
                    }
                    else if (y1>breastPt_y && y1<= bellyPt_y)
                    {
                        distance = distanceToLine(shoulderLeft_x3D,shoulderLeft_z3D,shoulderRight_x3D,shoulderRight_z3D,x3D_1,z3D_1);
                        volBelly+=(abs(x3D_1-x3D_2))*(abs(y3D_1-y3D_2))*distance;
                        numBellyVolume++;

                    }
                }
//                if (y1>breastPt_y && y1<= bellyPt_y)
//                {
////                    cout << x1 << " " << shoulderMidPoint << endl;
//                    if (x1==shoulderMidPoint)
//                    {
//                        out_YZ << "shoulderLeft = (" << shoulderLeft_y3D << "," << shoulderLeft_z3D << ")  shoulderRight = (" << shoulderRight_y3D << "," << shoulderRight_z3D << ")  Point = (" << y3D_1 << "," << z3D_1 << ")  bellyDistance = " << distance << endl;
//                    }

//                    if (y1==bellyPt_y-1)
//                    {
//                        outs << "shoulderLeft = (" << shoulderLeft_x3D << "," << shoulderLeft_z3D << ")  shoulderRight = (" << shoulderRight_x3D << "," << shoulderRight_z3D << ")  Point = (" << x3D_1 << "," << z3D_1 << ")  bellyDistance = " << distance << endl;
//                    }

//                    out3D << "Point = (" << x3D_1 << "," << y3D_1 << "," << z3D_1 << ")" << endl;

//                }

            }
        }


    }

//    cout << " numofPoints in Face Volume: " << numFaceVolume << endl;
//    cout << " numofPoints in Breast Volume: " << numBreastVolume << endl;
//    cout << " numofPoints in Belly Volume: " << numBellyVolume << endl;


//    file->close();
//    file_2->close();
//    file_3->close();

    volumes.push_back(volFace);
    volumes.push_back(volBreast);
    volumes.push_back(volBelly);



}

int* findValInBlob(const Vector<double> &data, cv::Mat blob, float yVal)
{
    int* xyPts = new int[2];
    bool foundPts = false;
    for (int i=0;i<blob.rows;i++)
    {
        int x = blob.at<float>(i,0);
        int y = blob.at<float>(i,1);

        int ind_vector = sub2ind(x,y,Globals::dImWidth);
//            cout << "index: " << ind_vector << endl;
        if (ind_vector >= data.getSize() || ind_vector < 0)
        {
            cout << "Error! " << ind_vector << " (" << x << "," << y << ") is out of range (0," << data.getSize()  << ")" << endl;
        }
        else
        {
//                cout << "(" << x << "," << y << ")" << endl;
            if ((data[ind_vector]> yVal - 0.05) && (data[ind_vector]< yVal + 0.05))
            {
                //cout << "(" << x << "," << y << ")" << endl;
                xyPts[0] = x;
                xyPts[1] = y;
                foundPts = true;
                return xyPts;

            }
            

        }

    }

    if (!foundPts)
    {
	xyPts[0] = 0;
        xyPts[1] = 0;
        return xyPts;
    }

}


void ReIdentifier::ExtractFeatures(PointCloudData pcloud, Vector< Vector< double> > detected_bounding_boxes, bool train)
{
//    pcloud.Z.writeToTXT("/home/scosar/Documents/Z_Data.txt");
//    pcloud.Y.writeToTXT("/home/scosar/Documents/Y_Data.txt");
//    pcloud.X.writeToTXT("/home/scosar/Documents/X_Data.txt");


    heightPeople.clear();
    widthShoulders.clear();
    lengthFaces.clear();

    volFacePeople.clear();
    volBreastPeople.clear();
    volBellyPeople.clear();

    headPtPeople.clear();
    feetPtPeople.clear();
    neckPtPeople.clear();
    shoulderPtsPeople.clear();
    segmentedBlobs.clear();

    featureExtracted.clear();

    facePartPtPeople.clear();
    breastPartPtPeople.clear();
    bellyPartPtPeople.clear();

    head3DPtPeople.clear();
    facePart3DPtPeople.clear();
    breastPart3DPtPeople.clear();
    bellyPart3DPtPeople.clear();
    shoulder3DPtsPeople.clear();

//    vector<float> heightPerson, widthShoulder, lengthFace;
    vector<float> headPt, feetPt, neckPt;
    vector<float> facePartPt, breastPartPt, bellyPartPt;
    vector<float> head3DPt, facePart3DPt, breastPart3DPt, bellyPart3DPt;
    vector<float> shoulderLeft, shoulderRight;
    vector<vector<float> > shoulderPts;

//    ROS_INFO("size bbox:%i", detected_bounding_boxes.getSize());
    for (int ibbox=0; ibbox < detected_bounding_boxes.getSize(); ibbox++)
    {
        float heightP,widthS,lengthF;
        headPt.clear();
        feetPt.clear();
        neckPt.clear();
        shoulderLeft.clear();
        shoulderRight.clear();
        shoulderPts.clear();

        facePartPt.clear();
        breastPartPt.clear();
        bellyPartPt.clear();

        head3DPt.clear();
        facePart3DPt.clear();
        breastPart3DPt.clear();
        bellyPart3DPt.clear();


//        ROS_INFO("size bbox[ibbox]:%i", detected_bounding_boxes(ibbox).getSize());

        int x_min = detected_bounding_boxes(ibbox)(0);
        int y_min = detected_bounding_boxes(ibbox)(1);
        int x_max = x_min + detected_bounding_boxes(ibbox)(2);
        int y_max = y_min + detected_bounding_boxes(ibbox)(3);

//        ROS_INFO("bbox xmin:%i, ymin:%i, xmax:%i, ymax:%i", x_min, y_min, x_max, y_max);

        if (y_min<0 && y_min > -30)
            y_min=0;
        if (x_min<0 && x_min > -30)
            x_min=0;
        if (y_max>Globals::dImHeight)
            y_max=Globals::dImHeight;
        if (x_max>Globals::dImWidth)
            x_max=Globals::dImWidth;

        if (x_min>=0 && y_min>=0 && x_max>=0 && y_max>=0)
        {
            int patchWidth = x_max-x_min+1;
            int patchHeight = y_max-y_min+1;
//            std::cout << patchWidth << "," << patchHeight <<  std::endl;
//            cv::Mat bbox_patch(patchHeight,patchWidth,CV_32FC1);
            cv::Mat bbox_patch_1D(patchHeight*patchWidth,1,CV_32FC1);

            //        Matrix bbox_patch(x_max-x_min+1,y_max-y_min+1,0);

//            float maxVal=0;
            int id=0;
            float depthPerson = 0;
            for (int x = x_min; x < x_max+1; x++)
            {
                for (int y = y_min; y < y_max+1; y++)
                {
                    int id_ptcloud = sub2ind(x,y,Globals::dImWidth);
//                    std::cout << "id_ptcloud=" << id_ptcloud << std::endl;
//                    bbox_patch.at<float>(y-y_min,x-x_min) = (float)pcloud.Z[id_ptcloud];
                    bbox_patch_1D.at<float>(id) = (float)pcloud.Z[id_ptcloud];
                    id++;
//                    bbox_patch.at<float>(x-x_min,y-y_min) = 5;
//                    std::cout << (float)pcloud.Z[id_ptcloud] << std::endl;

//                    if (maxVal<bbox_patch.at<float>(y-y_min,x-x_min))
//                    {
//                        maxVal = bbox_patch.at<float>(y-y_min,x-x_min);
//                    }

                    if ((y==y_max) && (x==x_min+round(patchWidth/2)) )
                    {
                        depthPerson = (float)pcloud.Z[id_ptcloud];
                    }
                }
            }
//            PrintMatrix(bbox_patch_1D);
//            writeTXTasCSV("/home/scosar/Documents/bbox_patch_1D.txt",bbox_patch_1D);
            float depthThrMax,depthThrMin;
            // !!Very Simple Thresholding!!
            depthThrMax = depthPerson + 0.5;
            depthThrMin = depthPerson - 0.5;

//            int histSize = 11;
////            float range[] = { 0,  maxVal } ;
////            const float* histRange = { range };
////            cv::Mat hist_bbox(histSize,2,CV_32FC1);

////            int id=0;
////            for (int i=0;i<bbox_patch.rows;i++)
////            {
////                for (int j=0;j<bbox_patch.cols;j++)
////                {
////                    bbox_patch_1D.at<float>(id) = bbox_patch.at<float>(i,j);
////                    id++;
////                }
////            }
////            cv::calcHist(&bbox_patch,1,0,cv::Mat(),hist_bbox,1,&histSize,&histRange);
//            calcHistogram(bbox_patch_1D,&hist_bbox,histSize);


//            // Assuming half of the bbox will be background, remove the background pixels
//            float histThrMax = 0.5; //(bbox_patch_1D.rows) / 2;
//            int idxMax=-1;
//            for (int i=0;i<hist_bbox.rows;i++)
//            {
//                if (hist_bbox.at<float>(i,0)>histThrMax)
//                {
//                    idxMax = i;
//                    break;
//                }
//            }
////            cout << "idxMax: "  << idxMax << endl;

//            // Find the 2nd peak point
//            cv::Mat new_hist_bbox = hist_bbox;
//            new_hist_bbox.at<float>(idxMax,0) = 0;

//            int idxMax2nd=-1;
//            float maxValHist=0;
//            for (int i=0;i<new_hist_bbox.rows;i++)
//            {
//                if (new_hist_bbox.at<float>(i,0)>maxValHist)
//                {
//                    maxValHist = new_hist_bbox.at<float>(i,0);
//                    idxMax2nd = i;
//                }
//            }

//            // Check if the person is in an interval of the 2nd peak point
//            // If not, find the 3rd peak point
//            float maxValHist_bin = new_hist_bbox.at<float>(idxMax2nd,1);
////            cout << "depthPerson: " << depthPerson << endl;
////            cout << "maxValHist: " << maxValHist << endl;
//            bool foundPerson=false;
//            if (depthPerson<(maxValHist_bin+(0.1*maxValHist_bin)) && depthPerson>(maxValHist_bin-(0.1*maxValHist_bin)) )
//            {
//                foundPerson=true;
//            }

//            if (foundPerson)
//            {
////                cout << "idxMax2nd: " << idxMax2nd << endl;
////                cout << "histVal: " << hist_bbox.at<float>(idxMax2nd,1) << endl;

////                depthThrMax = hist_bbox.at<float>(idxMax2nd,1) + (hist_bbox.at<float>(idxMax2nd,1) * 0.10);
////                depthThrMin = hist_bbox.at<float>(idxMax2nd,1) - (hist_bbox.at<float>(idxMax2nd,1) * 0.10);
//                depthThrMax = hist_bbox.at<float>(idxMax2nd,1) + 0.5;
//                depthThrMin = hist_bbox.at<float>(idxMax2nd,1) - 0.5;
//            }
//            else
//            {
//                cv::Mat new_hist_bbox_2 = new_hist_bbox;
//                new_hist_bbox_2.at<float>(idxMax2nd,0) = 0;

//                int idxMax3rd=-1;
//                float maxValNewHist=0;
//                for (int i=0;i<new_hist_bbox_2.rows;i++)
//                {
//                    if (new_hist_bbox_2.at<float>(i,0)>maxValNewHist)
//                    {
//                        maxValNewHist = new_hist_bbox_2.at<float>(i,0);
//                        idxMax3rd = i;
//                    }
//                }

////                cout << "idxMax3rd: " << idxMax3rd << endl;
////                cout << "histVal: " << hist_bbox.at<float>(idxMax2nd,1) << endl;

////                depthThrMax = hist_bbox.at<float>(idxMax3rd,1) + (hist_bbox.at<float>(idxMax3rd,1) * 0.10);
////                depthThrMin = hist_bbox.at<float>(idxMax3rd,1) - (hist_bbox.at<float>(idxMax3rd,1) * 0.10);
//                depthThrMax = hist_bbox.at<float>(idxMax3rd,1) + 0.5;
//                depthThrMin = hist_bbox.at<float>(idxMax3rd,1) - 0.5;
//            }

//            PrintMatrix(hist_bbox.t());


//            cout << "[" << depthThrMin << " , " << depthThrMax << "]" << endl;
//            ROS_INFO("depthThrMin: %f, depthThrMax: %f", depthThrMin,depthThrMax);

            // Segmentation
//            cv::Mat depthMap(Globals::dImHeight,Globals::dImWidth,CV_32FC1);
            cv::Mat sil_temp(Globals::dImHeight,Globals::dImWidth,CV_32FC1);
//            PrintMatrix(sil_depthMap);
            VectorToCvMatandThreshold(pcloud.Z,&sil_temp,depthThrMin, depthThrMax);
            int bigbox_min_x = x_min-(patchWidth/2);
            int bigbox_max_x = x_max+(patchWidth/2);
            if (bigbox_min_x<0)
                    bigbox_min_x=0;
            if (bigbox_max_x>=Globals::dImWidth)
                    bigbox_max_x=Globals::dImWidth-1;
//            cout << "bigbox_min_x: " << bigbox_min_x << endl;
//            cout << "bigbox_max_x: " << bigbox_max_x << endl;

            cv::Mat sil_depthMap = sil_temp.colRange(bigbox_min_x,bigbox_max_x);
//            writeTXTasCSV("/home/scosar/Documents/sil_depthMap.txt",sil_depthMap);
//            cout << "sil_depthMap-> width: " << sil_depthMap.cols << " height:" <<  endl;

//            writeTXTasCSV("./data/sil_temp.txt",sil_temp);
            sil_temp.release();

//            cv::threshold(depthMap,sil_depthMap,depthThrMax,1,cv::THRESH_BINARY);
//            cv::threshold(sil_depthMap,sil_depthMap,depthThrMin,1,cv::THRESH_TOZERO);

//            PrintMatrix(sil_depthMap);

            vector<cv::Mat> blobs;

            ConnectedComponentAnalysis(sil_depthMap,&blobs, bigbox_min_x);

//            writeTXTasCSV("./data/sil_depthMap_aftercca_" + QString::number(fNo) +  ".txt",sil_depthMap);
//            writeTXTasCSV("/home/scosar/Documents/sil_depthMap_aftercca.txt",sil_depthMap);
//            cv::Mat Y_Data(Globals::dImWidth,Globals::dImHeight,CV_32FC1);
//            VectorToCvMat(pcloud.Y,&Y_Data);

            // Estimate Height
            int head_y, head_x;
            bool objectIsPerson=false;
//            cout << "blob size: " << blobs.size() << endl;
            float headYLevel;

//            ROS_INFO("blobs size: %i", (int)blobs.size());
            int blobID=-1;
            for (int i=0; i<blobs.size(); i++)
            {
                cv::Mat blob = blobs[i];
//                cout << "blob row: " << blob.rows << endl;
//                cout << "sil_depthMap-> width: " << sil_depthMap.cols << " height: " << sil_depthMap.rows << endl;
//                cout << "sil_depthMap-> numel: " << sil_depthMap.rows*sil_depthMap.cols << endl;
//                ROS_INFO("blob row: %i, col: %i",blob.rows,blob.cols);
                if (blob.rows>(sil_depthMap.rows*sil_depthMap.cols)*0.05)
                {
                    featureExtracted.push_back(true);
                    blobID=i;
                    objectIsPerson=true;
                    segmentedBlobs.push_back(blob.clone());

                    cv::Mat blob_x = blob.col(0);
                    cv::Mat blob_y = blob.col(1);

//                    writeTXTasCSV("./data/blob_x_" + QString::number(fNo) + ".txt",blob_x);
//                    writeTXTasCSV("./data/blob_y_" + QString::number(fNo) + ".txt",blob_y);

                    double x_min_y,min_y,x_max_y,max_y;
//                    int id_min_x, id_min_y, id_max_x, id_max_y;
                    cv::Point id_min_y, id_max_y;
//                    cv::minMaxLoc(blob_x,&min_x,&max_x,&id_min_x,&id_max_x);
                    cv::minMaxLoc(blob_y,&min_y,&max_y,&id_min_y,&id_max_y);
//                    cv::minMaxIdx(blob_x,&min_x,&max_x);
//                    cv::minMaxIdx(blob_y,&min_y,&max_y);

                    x_min_y = blob_x.at<float>(id_min_y.y,0);
                    x_max_y = blob_x.at<float>(id_max_y.y,0);

//                    cout << "minx: " << min_x << ", maxx: " << max_x << ", miny: " << min_y << ", maxy: " << max_y << endl;
                    head_y = min_y;
                    head_x = x_min_y;
//                    int pt_x = round((min_x + max_x)/2);

//                    cout << "headPt: (" << x_min_y << "," << head_y << ")" << endl;
//                    cout << "feetPt: (" << x_max_y << "," << max_y << ")" << endl;

                    headYLevel = pcloud.Y[sub2ind(x_min_y,min_y,Globals::dImWidth)];
//                    ROS_INFO("headYLevel: %f",headYLevel);
//                    heightP = abs(pcloud.Y[sub2ind(x_min_y,min_y,Globals::dImWidth)] - pcloud.Y[sub2ind(x_max_y,max_y,Globals::dImWidth)]);
                    heightP = abs(pcloud.Y[sub2ind(x_min_y,min_y,Globals::dImWidth)] - gpLevel); // Use Y level of GP to measure height

//                    cout << "height: " << heightP << endl;
//                    ROS_INFO("height: %f, gpLevel: %f",heightP, gpLevel);

                    heightPeople.push_back(heightP);
                    headPt.push_back(x_min_y);
                    headPt.push_back(head_y);
                    feetPt.push_back(x_max_y);
                    feetPt.push_back(max_y);

                    float head3DPt_x = pcloud.X[sub2ind(x_min_y,min_y,Globals::dImWidth)];
                    float head3DPt_y = pcloud.Y[sub2ind(x_min_y,min_y,Globals::dImWidth)];
                    float head3DPt_z = pcloud.Z[sub2ind(x_min_y,min_y,Globals::dImWidth)];
                    head3DPt.push_back(head3DPt_x);
                    head3DPt.push_back(head3DPt_y);
                    head3DPt.push_back(head3DPt_z);

                }

            }

            if (blobID==-1)
            {

                featureExtracted.push_back(false);
                // For small blob fill with zeros
                heightPeople.push_back(0);
                headPt.push_back(0);
                headPt.push_back(0);
                feetPt.push_back(0);
                feetPt.push_back(0);
//                headPtPeople.push_back(headPt);
//                feetPtPeople.push_back(feetPt);
//                neckPtPeople.push_back(neckPt);

//                shoulderLeft.push_back(0);
//                shoulderLeft.push_back(0);
//                shoulderRight.push_back(0);
//                shoulderRight.push_back(0);
//                shoulderPts.push_back(shoulderLeft);
//                shoulderPts.push_back(shoulderRight);
//                shoulderPtsPeople.push_back(shoulderPts);

                head3DPt.push_back(0);
                head3DPt.push_back(0);
                head3DPt.push_back(0);

                cv::Mat dummyBlob;
                segmentedBlobs.push_back(dummyBlob);

            }

            // Estimate Distance Between Shoulders
            cv::Mat shoulder_line = sil_depthMap.row(y_max);
//            PrintMatrix(shoulder_line);
            int firstId=-1;
            int lastId;
            for (int i=x_min-bigbox_min_x; i<x_max-bigbox_min_x; i++)
            {
                if (shoulder_line.at<float>(i)>0)
                {
                    if (firstId==-1)
                        firstId=i+bigbox_min_x;
                    else
                        lastId=i+bigbox_min_x;
                }
            }
            widthS = abs(pcloud.X[sub2ind(lastId,y_max,Globals::dImWidth)] - pcloud.X[sub2ind(firstId,y_max,Globals::dImWidth)]);
//            cout << "shoulderLeftPt: (" << firstId << "," << y_max << ")" << endl;
//            cout << "shoulderRightPt: (" << lastId << "," << y_max << ")" << endl;

            float depthRight = abs(pcloud.Z[sub2ind(lastId,y_max,Globals::dImWidth)] - depthPerson);
            float depthLeft = abs(pcloud.Z[sub2ind(firstId,y_max,Globals::dImWidth)] - depthPerson);
            //ROS_INFO("depthRight: %f, depthLeft: %f",depthRight,depthLeft);

//            cout << "width: " << widthS << endl;
//            ROS_INFO("width: %f",widthS);
            widthShoulders.push_back(widthS);

            shoulderLeft.push_back(firstId);
            shoulderLeft.push_back(y_max);
            shoulderRight.push_back(lastId);
            shoulderRight.push_back(y_max);

            shoulderPts.push_back(shoulderLeft);
            shoulderPts.push_back(shoulderRight);

            // Estimage Lenght of Face
            int mid_x = round((lastId+firstId)/2);
//            ROS_INFO("bigbox_min_x: %i, bigbox_max_x: %i, x_min: %i, x_max: %i", bigbox_min_x, bigbox_max_x, x_min,x_max);
            int x_max_new = x_max;
            if (x_max+(patchWidth/2)>=Globals::dImWidth)
                    x_max_new = Globals::dImWidth-1;
//            ROS_INFO("bigbox_min_x: %i, bigbox_max_x: %i, x_min: %i, x_max: %i", bigbox_min_x, bigbox_max_x, x_min,x_max_new);
            cv::Mat sil_bbox_patch = sil_depthMap.colRange(x_min-bigbox_min_x,x_max_new-bigbox_min_x);
            sil_bbox_patch = sil_bbox_patch.rowRange(y_min,y_max);
            cv::Mat widths(sil_bbox_patch.rows,1,CV_32FC1);
//            cv::reduce(sil_bbox_patch, widths,1,CV_REDUCE_SUM);

//            PrintMatrix(sil_bbox_patch);
//            writeTXTasCSV("./data/sil_bbox_patch.txt",sil_bbox_patch);
            sumOverDim(&sil_bbox_patch,&widths,0);

//            PrintMatrix(widths);

            int upper_limit = round(detected_bounding_boxes[ibbox][3]/3); // Remove the first 1/3 of the bounding box to avoid forehead
            double minVal;
            int neck_id;
//            cout << upper_limit << " " << widths.rows-1 << endl;

            cv::Mat width_trimmed = widths.rowRange(upper_limit,widths.rows-1);
//            PrintMatrix(width_trimmed);
            cv::minMaxIdx(width_trimmed,&minVal,NULL,&neck_id);

//            cout << neck_id << endl;
            int neck_y = y_min + upper_limit + neck_id - 2;

//            cout << lastId << " " << firstId << endl;
//            ROS_INFO_STREAM("mid_x:" << mid_x << " neck_y:" << neck_y);
//            float neckYLevel = pcloud.Y[sub2ind(mid_x,neck_y,Globals::dImWidth)];
//            ROS_INFO("neckYlevel: %f",neckYLevel);
//            lengthF = abs(pcloud.Y[sub2ind(mid_x,neck_y,Globals::dImWidth)] - pcloud.Y[sub2ind(mid_x,head_y,Globals::dImWidth)]);
            lengthF = abs(pcloud.Y[sub2ind(mid_x,neck_y,Globals::dImWidth)] - headYLevel);

//            cout << "neckPt: (" << mid_x << "," << neck_y << ")" << endl;

//            cout << "face: " << lengthF << endl;
//            ROS_INFO("face: %f",lengthF);
            lengthFaces.push_back(lengthF);

            neckPt.push_back(mid_x);
            neckPt.push_back(neck_y);

            vector<float> volFeatures;
            if (blobID>=0)
            {

                float neckYLevel = headYLevel + 0.25;
                float breastYLevel = headYLevel + 0.65;
                float bellyYLevel = headYLevel + 1;

//                int neck_y_new = findValAlongYAxis(pcloud.Y, head_x,neckYLevel);
//                int breast_y = findValAlongYAxis(pcloud.Y, head_x,breastYLevel);
//                int belly_y = findValAlongYAxis(pcloud.Y, head_x,bellyYLevel);
                int* neck_newPts = findValInBlob(pcloud.Y, blobs.at(blobID), neckYLevel);
                int* breastPts = findValInBlob(pcloud.Y, blobs.at(blobID), breastYLevel);
                int* bellyPts = findValInBlob(pcloud.Y,blobs.at(blobID),bellyYLevel);

                facePartPt.push_back(neck_newPts[0]);
                facePartPt.push_back(neck_newPts[1]);
                breastPartPt.push_back(breastPts[0]);
                breastPartPt.push_back(breastPts[1]);
                bellyPartPt.push_back(bellyPts[0]);
                bellyPartPt.push_back(bellyPts[1]);


                calculateVolume(pcloud,blobs[blobID],firstId,lastId,head_y,neck_newPts[1],breastPts[1],bellyPts[1],volFeatures);

                //cout << "volFace: " << volFeatures[0] << ", volBreast: " << volFeatures[1] << ", volBelly: " << volFeatures[2] << endl;

                volFacePeople.push_back(volFeatures[0]);
                volBreastPeople.push_back(volFeatures[1]);
                volBellyPeople.push_back(volFeatures[2]);

                facePartPtPeople.push_back(facePartPt);
                breastPartPtPeople.push_back(breastPartPt);
                bellyPartPtPeople.push_back(bellyPartPt);

                float facePart3DPt_x = pcloud.X[sub2ind(facePartPt[0],facePartPt[1],Globals::dImWidth)];
                float facePart3DPt_y = pcloud.Y[sub2ind(facePartPt[0],facePartPt[1],Globals::dImWidth)];
                float facePart3DPt_z = pcloud.Z[sub2ind(facePartPt[0],facePartPt[1],Globals::dImWidth)];
                facePart3DPt.push_back(facePart3DPt_x);
                facePart3DPt.push_back(facePart3DPt_y);
                facePart3DPt.push_back(facePart3DPt_z);
                facePart3DPtPeople.push_back(facePart3DPt);

                float breastPart3DPt_x = pcloud.X[sub2ind(breastPartPt[0],breastPartPt[1],Globals::dImWidth)];
                float breastPart3DPt_y = pcloud.Y[sub2ind(breastPartPt[0],breastPartPt[1],Globals::dImWidth)];
                float breastPart3DPt_z = pcloud.Z[sub2ind(breastPartPt[0],breastPartPt[1],Globals::dImWidth)];
                breastPart3DPt.push_back(breastPart3DPt_x);
                breastPart3DPt.push_back(breastPart3DPt_y);
                breastPart3DPt.push_back(breastPart3DPt_z);
                breastPart3DPtPeople.push_back(breastPart3DPt);

                float bellyPart3DPt_x = pcloud.X[sub2ind(bellyPartPt[0],bellyPartPt[1],Globals::dImWidth)];
                float bellyPart3DPt_y = pcloud.Y[sub2ind(bellyPartPt[0],bellyPartPt[1],Globals::dImWidth)];
                float bellyPart3DPt_z = pcloud.Z[sub2ind(bellyPartPt[0],bellyPartPt[1],Globals::dImWidth)];
                bellyPart3DPt.push_back(bellyPart3DPt_x);
                bellyPart3DPt.push_back(bellyPart3DPt_y);
                bellyPart3DPt.push_back(bellyPart3DPt_z);
                bellyPart3DPtPeople.push_back(bellyPart3DPt);


            }
            else
            {
                facePartPt.push_back(0);
                facePartPt.push_back(0);
                breastPartPt.push_back(0);
                breastPartPt.push_back(0);
                bellyPartPt.push_back(0);
                bellyPartPt.push_back(0);

                facePartPtPeople.push_back(facePartPt);
                breastPartPtPeople.push_back(breastPartPt);
                bellyPartPtPeople.push_back(bellyPartPt);

                vector<float> shoulderLeft3D, shoulderRight3D;
                shoulderLeft3D.push_back(0);
                shoulderLeft3D.push_back(0);
                shoulderRight3D.push_back(0);
                shoulderRight3D.push_back(0);
                vector<vector<float> > shoulder3DPts;
                shoulder3DPts.push_back(shoulderLeft3D);
                shoulder3DPts.push_back(shoulderRight3D);
                shoulder3DPtsPeople.push_back(shoulder3DPts);

                facePart3DPt.push_back(0);
                facePart3DPt.push_back(0);
                facePart3DPt.push_back(0);
                facePart3DPtPeople.push_back(facePart3DPt);

                breastPart3DPt.push_back(0);
                breastPart3DPt.push_back(0);
                breastPart3DPt.push_back(0);
                breastPart3DPtPeople.push_back(breastPart3DPt);

                bellyPart3DPt.push_back(0);
                bellyPart3DPt.push_back(0);
                bellyPart3DPt.push_back(0);
                bellyPart3DPtPeople.push_back(bellyPart3DPt);

                volFacePeople.push_back(0);
                volBreastPeople.push_back(0);
                volBellyPeople.push_back(0);
            }

            headPtPeople.push_back(headPt);
            feetPtPeople.push_back(feetPt);
            neckPtPeople.push_back(neckPt);
            shoulderPtsPeople.push_back(shoulderPts);

            head3DPtPeople.push_back(head3DPt);
        } // if
        else
        {
            featureExtracted.push_back(false);
            // For small blob fill with zeros
            heightPeople.push_back(0);
            lengthFaces.push_back(0);
            widthShoulders.push_back(0);
            headPt.push_back(0);
            headPt.push_back(0);
            feetPt.push_back(0);
            feetPt.push_back(0);
            neckPt.push_back(0);
            neckPt.push_back(0);
            headPtPeople.push_back(headPt);
            feetPtPeople.push_back(feetPt);
            neckPtPeople.push_back(neckPt);
            shoulderLeft.push_back(0);
            shoulderLeft.push_back(0);
            shoulderRight.push_back(0);
            shoulderRight.push_back(0);
            shoulderPts.push_back(shoulderLeft);
            shoulderPts.push_back(shoulderRight);
            shoulderPtsPeople.push_back(shoulderPts);

            vector<float> shoulderLeft3D, shoulderRight3D;
            shoulderLeft3D.push_back(0);
            shoulderLeft3D.push_back(0);
            shoulderRight3D.push_back(0);
            shoulderRight3D.push_back(0);
            vector<vector<float> > shoulder3DPts;
            shoulder3DPts.push_back(shoulderLeft3D);
            shoulder3DPts.push_back(shoulderRight3D);
            shoulder3DPtsPeople.push_back(shoulder3DPts);

            facePartPt.push_back(0);
            facePartPt.push_back(0);
            breastPartPt.push_back(0);
            breastPartPt.push_back(0);
            bellyPartPt.push_back(0);
            bellyPartPt.push_back(0);

            facePartPtPeople.push_back(facePartPt);
            breastPartPtPeople.push_back(breastPartPt);
            bellyPartPtPeople.push_back(bellyPartPt);

            head3DPt.push_back(0);
            head3DPt.push_back(0);
            head3DPt.push_back(0);
            head3DPtPeople.push_back(head3DPt);
            facePart3DPt.push_back(0);
            facePart3DPt.push_back(0);
            facePart3DPt.push_back(0);
            facePart3DPtPeople.push_back(facePart3DPt);
            breastPart3DPt.push_back(0);
            breastPart3DPt.push_back(0);
            breastPart3DPt.push_back(0);
            breastPart3DPtPeople.push_back(breastPart3DPt);
            bellyPart3DPt.push_back(0);
            bellyPart3DPt.push_back(0);
            bellyPart3DPt.push_back(0);
            bellyPart3DPtPeople.push_back(bellyPart3DPt);

            cv::Mat dummyBlob;
            segmentedBlobs.push_back(dummyBlob);

            volFacePeople.push_back(0);
            volBreastPeople.push_back(0);
            volBellyPeople.push_back(0);
        }

    }

//    heightPeople.push_back(heightPerson);
//    widthShoulders.push_back(widthShoulder);
//    lengthFaces.push_back(lengthFace);

    if (train)
    {
        heightPeopleALL.push_back(heightPeople);
        widthShouldersALL.push_back(widthShoulders);
        lengthFacesALL.push_back(lengthFaces);
        volFacePeopleALL.push_back(volFacePeople);
        volBreastPeopleALL.push_back(volBreastPeople);
        volBellyPeopleALL.push_back(volBellyPeople);
    }

//    for(int i =0; i<headPtPeople.size();i++)
//    {
//        vector<float> headPts = headPtPeople.at(i);
//        cout << "size headPtPeople[i]: " << headPts.size() << endl;
//        vector<float> feetPts = feetPtPeople.at(i);
//        cout << "size feetPtPeople[i]: " << feetPts.size() << endl;
//        vector<float> neckPts = neckPtPeople.at(i);
//        cout << "size neckPtPeople[i]: " << neckPts.size() << endl;
//        vector<vector<float> > shoulderPts = shoulderPtsPeople.at(i);
//        cout << "size shouldertPtsPeople[i]: " << shoulderPts.size() << endl;
//        vector<float> shoulderLeft = shoulderPts.at(0);
//        cout << "size shouldertPtsPeople[i][0]: " << shoulderLeft.size() << endl;
//        vector<float> shoulderRight = shoulderPts.at(1);
//        cout << "size shouldertPtsPeople[i][1]: " << shoulderRight.size() << endl;
//        cout << "headPtPeople x: "<< headPts.at(0) << endl;
//        cout << "headPtPeople y: "<< headPts[1] << endl;
//        cout << "feetPtPeople x: " << feetPts[0] << endl;
//        cout << "feetPtPeople y: " << feetPts[1] << endl;
//        cout << "neckPtPeople x: " << neckPts[0] << endl;
//        cout << "neckPtPeople y: " << neckPts[1] << endl;
//        cout << "shoulderPtsPeople left_x: " << shoulderLeft[0] << endl;
//        cout << "shoulderPtsPeople left_y: " << shoulderLeft[1] << endl;
//        cout << "shoulderPtsPeople right_x: " << shoulderRight[0] << endl;
//        cout << "shoulderPtsPeople right_y: " << shoulderRight[1] << endl;
//    }

}

void ReIdentifier::LearnModels(int pNo)
{
    int noofSamples=0;
    vector<float> heightsOnePerson;
    vector<float> facesOnePerson;
    vector<float> shouldersOnePerson;
    vector<float> volFaceOnePerson;
    vector<float> volBreastOnePerson;
    vector<float> volBellyOnePerson;
    for (int i=0; i < heightPeopleALL.size(); i++)
    {
        vector<float> heights = heightPeopleALL.at(i);
        vector<float> faces = lengthFacesALL.at(i);
        vector<float> shoulders = widthShouldersALL.at(i);
        vector<float> volFaces = volFacePeopleALL.at(i);
        vector<float> volBreasts = volBreastPeopleALL.at(i);
        vector<float> volBellys = volBellyPeopleALL.at(i);

        if (heights.size()>0)
        {
            noofSamples++;
            float height = heights.at(pNo);
            float face = faces.at(pNo);
            float shoulder = shoulders.at(pNo);
            float volF = volFaces.at(pNo);
            float volBr = volBreasts.at(pNo);
            float volBe = volBellys.at(pNo);

            heightsOnePerson.push_back(height);
            facesOnePerson.push_back(face);
            shouldersOnePerson.push_back(shoulder);
            volFaceOnePerson.push_back(volF);
            volBreastOnePerson.push_back(volBr);
            volBellyOnePerson.push_back(volBe);

        }

    }

    cv::Mat allFeatures(noofSamples,6,CV_32FC1);
    for (int i=0; i < allFeatures.rows; i++)
    {

        float height = heightsOnePerson.at(i);
        float face = facesOnePerson.at(i);
        float shoulder = shouldersOnePerson.at(i);
        float volF = volFaceOnePerson.at(i);
        float volBr = volBreastOnePerson.at(i);
        float volBe = volBellyOnePerson.at(i);

        allFeatures.at<float>(i,0) = height;
        allFeatures.at<float>(i,1) = shoulder;
        allFeatures.at<float>(i,2) = face;
        allFeatures.at<float>(i,3) = volF;
        allFeatures.at<float>(i,4) = volBr;
        allFeatures.at<float>(i,5) = volBe;
    }

//    PrintMatrix(allFeatures);
    writeTXTasCSV(ros::package::getPath("reidentifier") + "/data/allFeatures.txt",allFeatures);

    cv::Mat covMatrix;
    cv::calcCovarMatrix(allFeatures,covMatrix,meanVector,CV_COVAR_ROWS | CV_COVAR_NORMAL,CV_32FC1);

    PrintMatrix(meanVector);
    PrintMatrix(covMatrix);

    cv::invert(covMatrix,icovMatrix,cv::DECOMP_SVD);

    PrintMatrix(icovMatrix);
}

void ReIdentifier::saveLearnedModel(string filename)
{

    ofstream file;
    file.open(filename.c_str());
//    QFile *file = new QFile(filename);
//    file->open(QIODevice::WriteOnly | QIODevice::Text);
//    QTextStream outs(file);

    file << "reidentifier:" << endl;
    if (personName.empty())
    {
        file << "   name: User" << endl;
    }
    else
    {
        file << "   name: " << personName << endl;
    }
    file << "   meanVector_rows: 1" << endl;
    file << "   meanVector_cols: 6" << endl;
    file << "   meanVector_dt: f" << endl;
    file << "   meanVector_data: [ " << meanVector.at<float>(0) << ", " << meanVector.at<float>(1) << ", " << meanVector.at<float>(2) << ", " << meanVector.at<float>(3) << ", "
         << meanVector.at<float>(4) << ", " << meanVector.at<float>(5) << " ]" << endl;
    file << "   icovMatrix_rows: 6" << endl;
    file << "   icovMatrix_cols: 6" << endl;
    file << "   icovMatrix_dt: f" << endl;
    file << "   icovMatrix_data: [ " << icovMatrix.at<float>(0,0) << ", " << icovMatrix.at<float>(0,1) << ", " << icovMatrix.at<float>(0,2) << ", " << icovMatrix.at<float>(0,3) << ", " << icovMatrix.at<float>(0,4) << ", " << icovMatrix.at<float>(0,5) << ", "
         << icovMatrix.at<float>(1,0) << ", " << icovMatrix.at<float>(1,1) << ", " << icovMatrix.at<float>(1,2) << ", " << icovMatrix.at<float>(1,3) << ", " << icovMatrix.at<float>(1,4) << ", " << icovMatrix.at<float>(1,5) << ", "
         << icovMatrix.at<float>(2,0) << ", " << icovMatrix.at<float>(2,1) << ", " << icovMatrix.at<float>(2,2) << ", " << icovMatrix.at<float>(2,3) << ", " << icovMatrix.at<float>(2,4) << ", " << icovMatrix.at<float>(2,5) << ", "
         << icovMatrix.at<float>(3,0) << ", " << icovMatrix.at<float>(3,1) << ", " << icovMatrix.at<float>(3,2) << ", " << icovMatrix.at<float>(3,3) << ", " << icovMatrix.at<float>(3,4) << ", " << icovMatrix.at<float>(3,5) << ", "
         << icovMatrix.at<float>(4,0) << ", " << icovMatrix.at<float>(4,1) << ", " << icovMatrix.at<float>(4,2) << ", " << icovMatrix.at<float>(4,3) << ", " << icovMatrix.at<float>(4,4) << ", " << icovMatrix.at<float>(4,5) << ", "
         << icovMatrix.at<float>(5,0) << ", " << icovMatrix.at<float>(5,1) << ", " << icovMatrix.at<float>(5,2) << ", " << icovMatrix.at<float>(5,3) << ", " << icovMatrix.at<float>(5,4) << ", " << icovMatrix.at<float>(5,5)
         << " ]" << endl;
    file.close();

}

void ReIdentifier::readLearnedModel(string name, std::vector<double> meanV, int meanVrows, int meanVcols, std::vector<double> icovM, int icovMrows, int icovMcols)
{
//    cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);

//    fs["name"] >> personName;
//    fs["meanVector"] >> meanVector;
//    fs["icovMatrix"] >> icovMatrix;

//    fs.release();
    personName = name;
    meanVector = cv::Mat(meanVrows,meanVcols,CV_32FC1);

    for (int j=0;j<meanVcols;j++)
    {
        meanVector.at<float>(j) = (float) meanV[j];
    }

    icovMatrix = cv::Mat(icovMrows,icovMcols,CV_32FC1);
    int numData=0;
    for (int i=0;i<icovMrows;i++)
    {
        for (int j=0;j<icovMcols;j++)
        {
            icovMatrix.at<float>(i,j) = (float) icovM[numData];
            numData++;
        }
    }


}

int ReIdentifier::Recognize()
{
    cv::Mat sample(1,6,CV_32FC1);

//    vector<float> heights = heightPeople.at(heightPeople.size()-1);
//    vector<float> faces = lengthFaces.at(lengthFaces.size()-1);
//    vector<float> shoulders = widthShoulders.at(widthShoulders.size()-1);

    distances.clear();
    double minDistance = 100;
    double dist;
    int matchId=-1;
    int maxSizeRecord = 0;
    int maxSizeId=-1;
//    cout << "size heightPeople: " << heightPeople.size() << endl;
//    cout << "size lengthFaces: " << lengthFaces.size() << endl;
//    cout << "size widthShoulders: " << widthShoulders.size() << endl;
//    cout << "size volFacePeople: " << volFacePeople.size() << endl;
//    cout << "size volBreastPeople: " << volBreastPeople.size() << endl;
//    cout << "size volBellyPeople: " << volBellyPeople.size() << endl;
    vector<float> stat, statHeight, statWidth, statLength, statVolF, statVolBr, statVolBe;
    vector<int> stableFeatureID;
    for (int pNo=0; pNo < heightPeople.size(); pNo++)
    {

        if (featureExtracted.at(pNo))
        {
            if ((lengthFaces.size()>pNo) && (widthShoulders.size()>pNo) )
            {
                float height = heightPeople.at(pNo);
                float face = lengthFaces.at(pNo);
                float shoulder = widthShoulders.at(pNo);
                float volFace = volFacePeople.at(pNo);
                float volBreast = volBreastPeople.at(pNo);
                float volBelly = volBellyPeople.at(pNo);

                sample.at<float>(0) = height;
                sample.at<float>(1) = shoulder;
                sample.at<float>(2) = face;
                sample.at<float>(3) = volFace;
                sample.at<float>(4) = volBreast;
                sample.at<float>(5) = volBelly;

                //        cout << "type meanVector: " << meanVector.type() << endl;
                //        cout << "type icovMatrix: " << icovMatrix.type() << endl;
                //        cout << "type sample: " << sample.type() << endl;

                //        PrintMatrix(meanVector);
                //        PrintMatrix(icovMatrix);
                //        PrintMatrix(sample);

                //cout << "meanVector: rows: " << meanVector.rows << " cols: " << meanVector.cols << endl;
                //cout << "sample: rows: " << sample.rows << " cols: " << sample.cols << endl;
                //cout << "icovMatrix: rows: " << icovMatrix.rows << " cols: " << icovMatrix.cols << endl;

                dist = cv::Mahalanobis(meanVector,sample,icovMatrix);
//                float a = calcMahalDistance(sample, meanVector,icovMatrix);
//                cout << "distance: " << dist << endl;
//                cout << "a: " << a << endl;
                distances.push_back((float)dist);

                vector<float> distanceP, heightP, widthP, lengthP, volFP, volBrP,volBeP;
                if (distancesBuffer.empty()) //Record the first person
                {

                    distanceP.push_back((float)dist);
                    distancesBuffer.push_back(distanceP);

                    heightP.push_back((float)height);
                    heightBuffer.push_back(heightP);

                    widthP.push_back((float)shoulder);
                    widthBuffer.push_back(widthP);

                    lengthP.push_back((float)face);
                    lengthBuffer.push_back(lengthP);

                    volFP.push_back((float)volFace);
                    volFaceBuffer.push_back(volFP);

                    volBrP.push_back((float)volBreast);
                    volBreastBuffer.push_back(volBrP);

                    volBeP.push_back((float)volBelly);
                    volBellyBuffer.push_back(volBeP);
                }
                else
                {
                    if (distancesBuffer.size()>pNo) // Find the record of the person pNo
                    {
                        distanceP = distancesBuffer.at(pNo);
                        distanceP.push_back((float)dist);
                        distancesBuffer[pNo] = distanceP;

                        heightP = heightBuffer.at(pNo);
                        heightP.push_back((float)height);
                        heightBuffer[pNo] = heightP;

                        widthP = widthBuffer.at(pNo);
                        widthP.push_back((float)shoulder);
                        widthBuffer[pNo] = widthP;

                        lengthP = lengthBuffer.at(pNo);
                        lengthP.push_back((float)face);
                        lengthBuffer[pNo] = lengthP;

                        volFP = volFaceBuffer.at(pNo);
                        volFP.push_back((float)volFace);
                        volFaceBuffer[pNo] = volFP;

                        volBrP = volBreastBuffer.at(pNo);
                        volBrP.push_back((float)volBreast);
                        volBreastBuffer[pNo] = volBrP;

                        volBeP = volBellyBuffer.at(pNo);
                        volBeP.push_back((float)volBelly);
                        volBellyBuffer[pNo] = volBeP;

                    }
                    else // Record the next person
                    {
                        distanceP.push_back((float)dist);
                        distancesBuffer.push_back(distanceP);

                        heightP.push_back((float)height);
                        heightBuffer.push_back(heightP);

                        widthP.push_back((float)shoulder);
                        widthBuffer.push_back(widthP);

                        lengthP.push_back((float)face);
                        lengthBuffer.push_back(lengthP);

                        volFP.push_back((float)volFace);
                        volFaceBuffer.push_back(volFP);

                        volBrP.push_back((float)volBreast);
                        volBreastBuffer.push_back(volBrP);

                        volBeP.push_back((float)volBelly);
                        volBellyBuffer.push_back(volBeP);
                    }
                }

                int sizeRecord = distancesBuffer[pNo].size();

                if (sizeRecord > maxSizeRecord)
                {
                    maxSizeRecord = sizeRecord;
                    maxSizeId = pNo;
//                    break;
                }

                //            if (dist > maxDistance)
                //            {
                //                maxDistance = dist;
                //                matchId = pNo;
                //            }

                if (distancesBuffer[pNo].size()==10)
                {
                    // Calculate
                   vector<float> distanceP = distancesBuffer.at(pNo);
                   vector<float> heightP = heightBuffer.at(pNo);
                   vector<float> widthP = widthBuffer.at(pNo);
                   vector<float> lengthP = lengthBuffer.at(pNo);
                   vector<float> volFP = volFaceBuffer.at(pNo);
                   vector<float> volBrP = volBreastBuffer.at(pNo);
                   vector<float> volBeP = volBellyBuffer.at(pNo);

                   calcMeanandStdDev(distanceP,&stat);
                   calcMeanandStdDev(heightP,&statHeight);
                   calcMeanandStdDev(widthP,&statWidth);
                   calcMeanandStdDev(lengthP,&statLength);
                   calcMeanandStdDev(volFP,&statVolF);
                   calcMeanandStdDev(volBrP,&statVolBr);
                   calcMeanandStdDev(volBeP,&statVolBe);
            //       distancesBuffer.clear();
                   heightP.erase(heightP.begin());
                   heightBuffer.at(pNo) = heightP;

                   widthP.erase(widthP.begin());
                   widthBuffer.at(pNo) = widthP;

                   lengthP.erase(lengthP.begin());
                   lengthBuffer.at(pNo) = lengthP;

                   volFP.erase(volFP.begin());
                   volFaceBuffer.at(pNo) = volFP;

                   volBrP.erase(volBrP.begin());
                   volBreastBuffer.at(pNo) = volBrP;

                   volBeP.erase(volBeP.begin());
                   volBellyBuffer.at(pNo) = volBeP;

                   distanceP.erase(distanceP.begin());
                   distancesBuffer.at(pNo) = distanceP;

//                   cout << "pNo: " << pNo << "stats: " << stat[0] << ", " << stat[1] << ", statsHeight: " << statHeight[0] << ", " << statHeight[1] << ", statsVolBe: " << statVolBe[0] << ", " << statVolBe[1] << endl ;
                   if (statHeight[1] < 0.2 && statVolBe[0] < 0.01)
                   {
                       stableFeatureID.push_back(pNo);
        //               if (stat[0]<1)
        //                   return maxSizeId;
                   }
                }

            }

        }
        else
        {
            distances.push_back(0);
        }



    } // for

    for (int i=0; i<stableFeatureID.size(); i++)
    {
        if (distances[stableFeatureID[i]] < minDistance)
        {
            minDistance = distances[stableFeatureID[i]];
            if (minDistance<=ridentify_thrs)
            {
                if (matchedIDs.size())
                {
                    matchId = matchedIDs[stableFeatureID[i]];
                }
                else
                {
                    matchId = stableFeatureID[i];
                }
            }
        }


    }

//    if (maxSizeRecord==10)
//    {
//        // Calculate
//       vector<float> distanceP = distancesBuffer.at(maxSizeId);
//       vector<float> heightP = heightBuffer.at(maxSizeId);
//       vector<float> widthP = widthBuffer.at(maxSizeId);
//       vector<float> lengthP = lengthBuffer.at(maxSizeId);
//       vector<float> volFP = volFaceBuffer.at(maxSizeId);
//       vector<float> volBrP = volBreastBuffer.at(maxSizeId);
//       vector<float> volBeP = volBellyBuffer.at(maxSizeId);

//       calcMeanandStdDev(distanceP,&stat);
//       calcMeanandStdDev(heightP,&statHeight);
//       calcMeanandStdDev(widthP,&statWidth);
//       calcMeanandStdDev(lengthP,&statLength);
//       calcMeanandStdDev(volFP,&statVolF);
//       calcMeanandStdDev(volBrP,&statVolBr);
//       calcMeanandStdDev(volBeP,&statVolBe);
////       distancesBuffer.clear();
//       heightP.erase(heightP.begin());
//       heightBuffer.at(maxSizeId) = heightP;

//       widthP.erase(widthP.begin());
//       widthBuffer.at(maxSizeId) = widthP;

//       lengthP.erase(lengthP.begin());
//       lengthBuffer.at(maxSizeId) = lengthP;

//       volFP.erase(volFP.begin());
//       volFaceBuffer.at(maxSizeId) = volFP;

//       volBrP.erase(volBrP.begin());
//       volBreastBuffer.at(maxSizeId) = volBrP;

//       volBeP.erase(volBeP.begin());
//       volBellyBuffer.at(maxSizeId) = volBeP;

//       distanceP.erase(distanceP.begin());
//       distancesBuffer.at(maxSizeId) = distanceP;

//       cout << "stats: " << stat[0] << ", " << stat[1] << ", statsHeight: " << statHeight[0] << ", " << statHeight[1] << ", statsVolBe: " << statVolBe[0] << ", " << statVolBe[1] << endl ;
//       if (statHeight[1] < 0.2 && statVolBe[0] < 0.01)
//       {
//           if (stat[0]<1)
//               return maxSizeId;
//       }
//    }



//    if (maxDistance>1.5) // No matching person
//        matchId = -1;

    return matchId;

}
