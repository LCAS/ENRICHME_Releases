#ifndef REIDENTIFIER_H
#define REIDENTIFIER_H

#include "Vector.h"
#include "Matrix.h"
#include "Camera.h"
#include "pointcloud.h"
#include "AncillaryMethods.h"
#include "cv.h"
#include "cxcore.h"

class ReIdentifier
{
public:
    ReIdentifier();
    void SetGPLevel(float gp_y);
    void SetThreshold(float thrs);
    void SetIDs(Vector<int>& IDs);
    void calculateVolume(PointCloudData pcloud, cv::Mat blob, float shoulderLeft_x, float shoulderRight_x, float headPt_y, float neckPt_y, float breastPt_y, float bellyPt_y, vector<float> &volumes);
    void ExtractFeatures(PointCloudData pcloud, Vector< Vector< double> > detected_bounding_boxes, bool train);
    void ConnectedComponentAnalysis(cv::Mat binaryImg, vector<cv::Mat> *blobs, int xOffset);
    void LearnModels(int pNo=0);
    void readLearnedModel(string name, std::vector<double> meanV, int meanVrows, int meanVcols, std::vector<double> icovM, int icovMrows, int icovMcols);
    void saveLearnedModel(string filename);
    int Recognize();

    vector<bool> featureExtracted;
    vector<vector<float> > headPtPeople, feetPtPeople, neckPtPeople;
    vector<vector<float> > facePartPtPeople, breastPartPtPeople, bellyPartPtPeople;
    vector<vector<vector<float> > > shoulderPtsPeople;
    vector<vector<vector<float> > > shoulder3DPtsPeople;
    vector<vector<float> > head3DPtPeople, facePart3DPtPeople, breastPart3DPtPeople, bellyPart3DPtPeople;
    vector<float> heightPeople, widthShoulders, lengthFaces, volFacePeople, volBreastPeople, volBellyPeople;
    vector<vector<float> > heightPeopleALL, widthShouldersALL, lengthFacesALL, volFacePeopleALL, volBreastPeopleALL, volBellyPeopleALL;
    cv::Mat meanVector;
    cv::Mat icovMatrix;
    string personName;
    vector<float> distances;
    float ridentify_thrs;
    vector<int> matchedIDs;
    vector<vector<float> > distancesBuffer, heightBuffer, widthBuffer, lengthBuffer, volFaceBuffer, volBreastBuffer, volBellyBuffer;
    float gpLevel;

    vector<cv::Mat> segmentedBlobs;
};

#endif // REIDENTIFIER_H
