#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <map>
#include <unordered_map>
#include <numeric>
#include <Eigen>
#include <time.h>
using namespace std;

// 平面
enum Plane
{
	XOY,
	XOZ,
	YOZ
};


// 空间直方图计算
// 法向量的视点坐标
struct Normal
{
	double x, y, z;
};
struct Point3D
{
	double x, y, z;
	Normal normal;
	double angle;
	// 重点：重载“<”，作为map的键
	bool operator<(const Point3D & p)const {
		return (x < p.x) || (x == p.x && y < p.y);
	}

};

struct Point2D
{
	float x, y;
	bool operator<(const Point2D & p)const {
		return (x < p.x) || (x == p.x && y < p.y);
	}

};

struct Boundary
{
	double xMin, xMax;
	double yMin, yMax;
	double zMin, zMax;
};

ostream & operator<<(ostream & out, Point2D A) {

	out << "[" << A.x << "," << A.y << "]";
	return out;
}

ostream & operator<<(ostream & out, Point3D A) {

	out << "[" << A.x << "," << A.y << "," << A.z << "]";
	return out;
}
ostream & operator<<(ostream & out, Normal A) {

	out << "[" << A.x << "," << A.y << "," << A.z << "]";
	return out;
}
// 读取文件点云数据
vector<Point2D> getData(const char* filename, Plane intestingPlane, vector<float> &boundaryValue) {
	
	FILE *fp_txt1;
	
	fp_txt1 = fopen(filename, "r");

	Point3D TxtPoint1;
	Point2D tempData;
	vector<Point2D> pointCloudData;

	vector<float> x_data, y_data;

	if (fp_txt1)
	{
		// 获取要统计的平面的点云
		switch (intestingPlane) {
		case XOY:
			while (fscanf(fp_txt1, "%lf %lf %lf", &TxtPoint1.x, &TxtPoint1.y, &TxtPoint1.z) != EOF)
			{
				
				tempData.x = TxtPoint1.x;
				x_data.push_back(TxtPoint1.x);
				y_data.push_back(TxtPoint1.y);
				tempData.y = TxtPoint1.y;
				pointCloudData.push_back(tempData);
			}
			break;
		case XOZ:
			while (fscanf(fp_txt1, "%lf %lf %lf", &TxtPoint1.x, &TxtPoint1.y, &TxtPoint1.z) != EOF)
			{

				tempData.x = TxtPoint1.x;
				tempData.y = TxtPoint1.z;
				x_data.push_back(TxtPoint1.x);
				y_data.push_back(TxtPoint1.z);
				pointCloudData.push_back(tempData);
			}
			break;
		case YOZ:
			while (fscanf(fp_txt1, "%lf %lf %lf", &TxtPoint1.x, &TxtPoint1.y, &TxtPoint1.z) != EOF)
			{

				tempData.x = TxtPoint1.y;
				tempData.y = TxtPoint1.z;
				x_data.push_back(TxtPoint1.y);
				y_data.push_back(TxtPoint1.z);
				pointCloudData.push_back(tempData);
			}
			break;
		default:
			break;
		}


	}
	else{

		cout << "txt数据加载失败！" << endl;
	}
	sort(x_data.begin(), x_data.end());
	sort(y_data.begin(), y_data.end());
	boundaryValue.push_back(x_data[0]);
	boundaryValue.push_back(x_data[x_data.size() - 1]);
	boundaryValue.push_back(y_data[0]);
	boundaryValue.push_back(y_data[y_data.size() - 1]);
	return pointCloudData;
}

// 统计各个单位点云数目,
map<Point2D, int> countPointCloud(vector<Point2D> pointCloudDatas, vector<float> boudaryData, int segNum) {
	// 区间数10*10

	float xSegValue = (boudaryData[1] - boudaryData[0]) / segNum;
	float ySegValue = (boudaryData[3] - boudaryData[2]) / segNum;
	//cout << "xSegValue: " << xSegValue << " ySegValue: " << ySegValue << endl;
	map<Point2D, int> mpPointData;
	int tenSum = 0;
	// 判断属于哪个区间，并统计该区间点云的个数
	for (Point2D & pointCloudData : pointCloudDatas) {
		Point2D tempData;

		// 值减去最小值再除以区间间隔值，判断属于哪个区间 
		float xTemp = (pointCloudData.x - boudaryData[0]) / xSegValue;
		float yTemp = (pointCloudData.y - boudaryData[2]) / ySegValue;
		tempData.x = floorf(xTemp);
		tempData.y = floorf(yTemp);
		if (tempData.x == segNum || tempData.y == segNum) {
			tenSum += 1;
			
		}
		else {
			mpPointData[tempData] += 1;
		}
		
	}
	int sum = 0;
	for (auto it = mpPointData.begin(); it != mpPointData.end(); it++) {
		sum += it->second;
	}
	return mpPointData;
}

// 计算相似度
float cacular2Similarity(map<Point2D,int> data1, map<Point2D,int> data2, int segNum) {
	// 存储100个单元的点云数
	vector<int> unitPointNum1(segNum * segNum, 0);
	vector<int> unitPointNum2(segNum * segNum, 0);
	// 临时存储当前遍历的位置
	Point2D curLoc;
	int k = 0;

	// 提取出100个单元中每个单元的点云数
	for (int i = 0; i < segNum; ++i) {
		for (int j = 0; j < segNum; ++j) {
			curLoc.x = i; 
			curLoc.y = j; 
			if (data1.find(curLoc) != data1.end()) {
				unitPointNum1[k] = data1[curLoc];
			}
			if (data2.find(curLoc) != data2.end()) {
				unitPointNum2[k] = data2[curLoc];
			}
			k++;
		}
	}

	int sumNum1 = accumulate(unitPointNum1.begin(), unitPointNum1.end(), 0);
	int sumNum2 = accumulate(unitPointNum2.begin(), unitPointNum2.end(), 0);
	double similarity = 0;
	for (int i = 0; i < segNum * segNum; ++i) {
		similarity += sqrt(((double)unitPointNum1[i] / sumNum1) * ((double)unitPointNum2[i] / sumNum2));
	}
	return similarity;
}

//二维平面栅格化后，相似度比较
void similarity2D(const char *billboardFile, const char *geometricFile) {

	vector<Plane> Planes{ XOY, XOZ, YOZ };
	vector<float> similarities(3);
	int segNum = 20;
	for (int i = 0; i < 3; i++)
	{
		vector<float> b_boudaryData;
		vector<float> g_boudaryData;
		vector<Point2D> billboardData = getData(billboardFile, Planes[i], b_boudaryData);
		vector<Point2D> geometricData = getData(geometricFile, Planes[i], g_boudaryData);

		// 2D网格点云数目统计信息
		auto billboardDataCount = countPointCloud(billboardData, b_boudaryData, segNum);
		auto geometricDataCount = countPointCloud(geometricData, g_boudaryData, segNum);

		// 计算相似度
		similarities[i] = cacular2Similarity(billboardDataCount, geometricDataCount, segNum);
		cout << similarities[i] << endl;
	}

	cout.setf(ios::fixed);
	cout.setf(ios::showpoint);
	cout.precision(4); 

	float average = (float)accumulate(similarities.begin(), similarities.end(), 0.0f) / 3;
	cout << "========================相似度信息=========================" << endl;
	for (float i : similarities) {
		cout << i << " ";
	}
	cout << average << endl;
}

// 获取三维点云的数据，遍历点云获得边界X、Y、Z的最大，最小值
 vector<Point3D> get3Data(const char* filename, Boundary &boundary) {

	 FILE *fp_txt1;

	 fp_txt1 = fopen(filename, "r");

	 Point3D TxtPoint;
	 vector<Point3D>pointData;
	 vector<double> x_data, y_data, z_data;
	 while (fscanf(fp_txt1, "%lf %lf %lf %lf %lf %lf", &TxtPoint.x, &TxtPoint.y, &TxtPoint.z, 
													   &TxtPoint.normal.x, &TxtPoint.normal.y, &TxtPoint.normal.z) != EOF)
	 {
		 x_data.push_back(TxtPoint.x);
		 y_data.push_back(TxtPoint.y);
		 z_data.push_back(TxtPoint.z);
		 pointData.push_back(TxtPoint);
	 }
	 cout << "=========================统计数据======================" << endl;
	 cout << "3D点云数据数目: " << pointData.size() << endl;

	 // 统计边界
	 sort(x_data.begin(), x_data.end());
	 sort(y_data.begin(), y_data.end());
	 sort(z_data.begin(), z_data.end());
	 boundary.xMin = x_data[0]; boundary.xMax = x_data[x_data.size() - 1];
	 boundary.yMin = y_data[0]; boundary.yMax = y_data[y_data.size() - 1];
	 boundary.zMin = z_data[0]; boundary.zMax = z_data[z_data.size() - 1];

	 return pointData;
}

 // 统计各个网格内的数据，网格化
map<Point3D, vector<Point3D>> countMeshData(vector<Point3D> &pointDatas, Boundary boundary, int meshNum) {
	
	double xMeshValue = (boundary.xMax - boundary.xMin) / meshNum;
	double yMeshValue = (boundary.yMax - boundary.yMin) / meshNum;
	double zMeshValue = (boundary.zMax - boundary.zMin) / meshNum;
	// 键为网格坐标，键值为该网格内的点云
	map<Point3D, vector<Point3D>> meshData;
	Point3D tempData;
	for (auto & pointData : pointDatas) {
		// 值减去最小值再除以区间间隔值，判断属于哪个区间
		double xTemp = (pointData.x - boundary.xMin) / xMeshValue;
		double yTemp = (pointData.y - boundary.yMin) / yMeshValue;
		double zTemp = (pointData.z - boundary.zMin) / zMeshValue;
		tempData.x = floor(xTemp);
		tempData.y = floor(yTemp);
		tempData.z = floor(zTemp);
		if (tempData.x == meshNum || tempData.y == meshNum || tempData.z == meshNum) {
			// 边界处理
			auto xtemp = tempData.x == meshNum ? tempData.x - 1 : tempData.x;
			tempData.x = xtemp;
			auto ytemp = tempData.y == meshNum ? tempData.y - 1 : tempData.y;
			tempData.y = ytemp;
			auto ztemp = tempData.z == meshNum ? tempData.z - 1 : tempData.z;
			tempData.z = ztemp;
		}
		// 数据存储
		meshData[tempData].push_back(pointData);
	}
	int pointSum = 0;
	cout << "网格数量：" << meshData.size() << endl;
	return meshData;
}



int main() {

	const char* billboardFile = "billboard.txt";
	const char* geometricFile = "geometric.txt";
	similarity2D(billboardFile, geometricFile);
	system("pause");
	return 0;
}