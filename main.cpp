#include <iostream>
#include <vector>

#include "opencv2/video/video.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

static bool runCalibration(const Size imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector<vector<Point2f>> imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs, vector<float>& reprojErrs, double& totalAvgErr);

const Size bSize(5, 4);
const int squareSize = 100;

const Point2f &getPoint(const vector<Point2f> &v, const Size &boardSize, int y, int x) {
	return v[y * boardSize.width + x];
}

const Point2f &getPointNew(const vector<Point2f> &v, int stride, int y, int x) {
	return v[y * stride + x];
}


Size calculateSizeToDisplay(const Size &original, int H = 1920, int W = 1080, float C = 2.2) {
	float hRatio = original.height / H;
	float wRatio = original.width / W;

	float ratio = fmax(hRatio, wRatio) + C;
	return Size(original.height / ratio, original.width / ratio);
}

Size calculateSizeToDisplay(const Mat &c, int H = 1920, int W = 1080, float C = 2.2) {
	vector<Point2f> corners = (vector<Point2f>)c;
	float minX = fmin(corners[0].x, corners[3].x);
	float maxX = fmax(corners[1].x, corners[2].x);
	float minY = fmin(corners[0].y, corners[1].y);
	float maxY = fmax(corners[2].y, corners[3].y);
	float newWidth = maxX - minX;
	float newHeight = maxY - minY;
	return calculateSizeToDisplay(Size(newHeight, newWidth), H, W, C);
}

inline void displayResult(const std::string &windowName, const Mat &result) {
	Mat resized;
	resize(result, resized, calculateSizeToDisplay(Size(result.rows, result.cols)));
	imshow(windowName, resized);
}

struct corners_info_t {
	corners_info_t() = delete;
	corners_info_t(const vector<Point2f> &corners) {
		minX = fmin(fmin(corners[0].x, corners[1].x), fmin(corners[2].x, corners[3].x));
		maxX = fmax(fmax(corners[0].x, corners[1].x), fmax(corners[2].x, corners[3].x));
		minY = fmin(fmin(corners[0].y, corners[1].y), fmin(corners[2].y, corners[3].y));
		maxY = fmax(fmax(corners[0].y, corners[1].y), fmax(corners[2].y, corners[3].y));
		width = maxX - minX;
		height = maxY - minY;
	}

	void print() {
		cout << "X: " << minX << " " << maxX << endl;
		cout << "Y: " << minY << " " << maxY << endl;
		cout << "H x W: " << height << " " << width << endl;
	}

	float minX, maxX, minY, maxY;
	float width, height;
};

void getImageCorners(const Mat &image, vector<Point2f> &corners) {
	corners.clear();
	corners.push_back(Point2f(0, image.rows)); // bottom left
	corners.push_back(Point2f(image.cols, image.rows)); // bottom right
	corners.push_back(Point2f(image.cols, 0)); // top right
	corners.push_back(Point2f(0, 0)); // top left
}

float eps = 0e-5;

// A is collinear to B : Ax * By == Ay * Bx
// A = b - a
// B = c - b
bool collinear(const Point2f &a, const Point2f &b, const Point2f &c) { //double x1, double y1, double x2, double y2, double x3, double y3) {
	cout << "collinear: " << a << " " << b << " " << c << endl;
	Point2f A(b.x - a.x, b.y - a.y);
	Point2f B(c.x - b.x, c.y - b.y);
	cout << "collinear: " << A << " " << B << endl;
	cout << "collinear: " << fabs(A.x * B.y - A.y * B.x) << endl;
	
	return fabs(A.x * B.y - A.y * B.x) <= 1e-6;
	//return fabs((a.y - b.y) * (a.x - c.x) - (a.y - c.y) * (a.x - b.x)) <= 1e-9;
}

void getSteps(bool byRow, const Size &bs, const Point2f &first, const Point2f &second, const Point2f &third, int &istart, int &iend, int &istep, int &jstart, int &jend, int &jstep) {
	if (byRow) {
		// A B
		// C
		// where A - first, B - second, C - third
		// AB - row
		// AC - column
		int w = (byRow) ? bs.width : bs.height;
		int h = (byRow) ? bs.height : bs.width;
		if (first.x < second.x) {
			// columns goes from left to right
			jstart = 0;
			jend = w;
			jstep = 1;
		}
		else {
			// columns goes from right to left
			jstart = w - 1;
			jend = -1;
			jstep = -1;
		}

		if (first.y < third.y) {
			// rows goes from top to bottom
			istart = 0;
			iend = h;
			istep = 1;
		}
		else {
			// rows goes from bottom to top
			istart = h - 1;
			iend = -1;
			istep = -1;
		}
	} else {
		// by column
		// A B
		// C
		// where A - first, B - second, C - third
		// AB - column
		// AC - row
		int w = (byRow) ? bs.width : bs.height;
		int h = (byRow) ? bs.height : bs.width;
		if (first.x > third.x) {
			// columns goes from left to right
			jstart = 0;
			jend = w;
			jstep = 1;
		}
		else {
			// columns goes from right to left
			jstart = w - 1;
			jend = -1;
			jstep = -1;
		}

		if (first.y < second.y) {
			// rows goes from top to bottom
			istart = 0;
			iend = h;
			istep = 1;
		}
		else {
			// rows goes from bottom to top
			istart = h - 1;
			iend = -1;
			istep = -1;
		}
	}
	
	cout << "byRow: " << byRow << endl;
	cout << "getSteps: " << first << " " << second << " " << third << endl;
	cout << "i: " << istart << " " << iend << " " << istep << endl;
	cout << "j: " << jstart << " " << jend << " " << jstep << endl;
}

void getSteps(const vector<Point2f> &p, const Size &bs, bool &byRow, int &istart, int &iend, int &istep, int &jstart, int &jend, int &jstep) {
	byRow = true;
	for (int i = 1; i < bs.width; ++i) {
		if (p[i - 1].x > p[i].x) {
			byRow = false;
		}
	}
	if (!byRow) {
		byRow = true;
		for (int i = 1; i < bs.width; ++i) {
			if (p[i - 1].x < p[i].x) {
				byRow = false;
			}
		}
	}

	if (byRow) {
		Point2f first = getPointNew(p, bs.width, 0, 0);
		Point2f second = getPointNew(p, bs.width, 0, 1);
		Point2f third = getPointNew(p, bs.width, 1, 0);
		getSteps(true, bs, first, second, third, istart, iend, istep, jstart, jend, jstep);
	} else {
		// by column
		Point2f first = getPointNew(p, bs.height, 0, 0);
		Point2f second = getPointNew(p, bs.height, 0, 1);
		Point2f third = getPointNew(p, bs.height, 1, 0);
		getSteps(false, bs, first, second, third, istart, iend, istep, jstart, jend, jstep);
	}
}

vector<vector<Point2f>> organize(const vector<Point2f> &p, const Size &bs) {
	bool isByRow = false;

	vector<vector<Point2f>> res(bs.height, vector<Point2f>(bs.width));
	int jstart, jend, jstep;
	int istart, iend, istep;
	getSteps(p, bs, isByRow, istart, iend, istep, jstart, jend, jstep);

	if (isByRow) {
		for (int i = 0, ii = istart; ii != iend; ++i, ii += istep) {
			for (int j = 0, jj = jstart; jj != jend; ++j, jj += jstep) {
				res[i][j] = getPointNew(p, bs.width, ii, jj);
			}
		}
	} else {
		for (int j = 0, jj = jstart; jj != jend; ++j, jj += jstep) {
			for (int i = 0, ii = istart; ii != iend; ++i, ii += istep) {
				res[i][j] = getPointNew(p, bs.height, ii, jj);
			}
		}
	}

	cout << "organize result: " << endl;
	for (auto &R : res) {
		cout << R << endl;
	}
	return res;
}

vector<vector<int>> splitByRows(const vector<Point2f> &p, float yrange = 100) {
	vector<vector<int>> res;
	vector<int> base;
	for (int i = 0; i < p.size(); ++i) {
		// find a corresponding base
		int bindex = -1;
		for (int j = 0; j < base.size(); ++j) {
			if (base[j] - yrange <= p[i].y && p[i].y <= base[j] + yrange) {
				bindex = j;
				break;
			}
		}

		if (bindex < 0) {
			bindex = base.size();
			base.push_back(p[i].y);
			res.resize(res.size() + 1);
		}
		res[bindex].push_back(i);
	}

	for (int i = 0; i < res.size(); ++i) {
		for (int j = 0; j < res[i].size(); ++j) {
			cout << i << ", " << j << " " << p[res[i][j]] << endl;
		}
	}

	return res;
}

pair<Point2f, Point2f> getTwoPointsNew(const vector<vector<Point2f>> &points) {
	return make_pair(points[points.size() - 1][0], points[points.size() - 1][1]);
}

pair<int, int> getTwoPoints(const vector<Point2f> &points) {
	pair<int, int> res;

	vector<vector<int>> byRows = splitByRows(points);
	
	int b = 0;
	for (int i = 1; i < byRows.size(); ++i) {
		if (points[byRows[i][0]].y > points[byRows[b][0]].y) {
			b = i;
		}
	}

	res.first = byRows[b][0];
	res.second = byRows[b][1];
	if (points[res.first].x > points[res.second].x)
		swap(res.first, res.second);

	for (int i = 2; i < byRows[b].size(); ++i) {
		if (points[byRows[b][i]].x < points[res.first].x) {
			res.second = res.first;
			res.first = i;
		}
		else if (points[byRows[b][i]].x < points[res.second].x) {
			res.second = i;
		}
	}

	return res;
}

vector<Point2f> extractCorners(const vector<Point2f> &points) {
	vector<vector<int>> byRows = splitByRows(points);

	int b = 0;
	for (int i = 1; i < byRows.size(); ++i) {
		if (points[byRows[i][0]].y > points[byRows[b][0]].y) {
			b = i;
		}
	}

	int bl = 0;
	int br = 0;
	for (int i = 0; i < byRows[b].size(); ++i) {
		if (points[byRows[b][i]].x < points[byRows[b][bl]].x)
			bl = i;
		if (points[byRows[b][i]].x > points[byRows[b][br]].x)
			br = i;
	}

	bl = byRows[b][bl];
	br = byRows[b][br];

	b = 0;
	for (int i = 1; i < byRows.size(); ++i) {
		if (points[byRows[i][0]].y < points[byRows[b][0]].y) {
			b = i;
		}
	}

	int tl = 0;
	int tr = 0;
	for (int i = 0; i < byRows[b].size(); ++i) {
		if (points[byRows[b][i]].x < points[byRows[b][tl]].x)
			tl = i;
		if (points[byRows[b][i]].x > points[byRows[b][tr]].x)
			tr = i;
	}

	tl = byRows[b][tl];
	tr = byRows[b][tr];

	vector<Point2f> res;
	res.push_back(points[bl]);
	res.push_back(points[br]);
	res.push_back(points[tr]);
	res.push_back(points[tl]);

	return res;
}

vector<Point2f> extractCornersNew(const vector<vector<Point2f>> &points) {
	vector<Point2f> res;
	int rows = points.size();
	int cols = points.front().size();
	res.push_back(points[rows - 1][0]); // bottom left
	res.push_back(points[rows - 1][cols - 1]); // bottom right
	res.push_back(points[0][cols - 1]); // top right
	res.push_back(points[0][0]); // top left
	return res;
}

bool projectToTheFloor(const Mat &image, const Size &boardSize, Mat &result, vector<Point2f> &rectangle, vector<Point2f> &corners) {
	// search for chessboard corners
	vector<Point2f> chessboardCorners;
	if (!findChessboardCorners(image, boardSize, chessboardCorners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE))
		return false;

	// optimize results
	Mat viewGray;
	cvtColor(image, viewGray, CV_BGR2GRAY);
	cornerSubPix(viewGray, chessboardCorners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

	Mat temp;
	image.copyTo(temp);
	drawChessboardCorners(temp, boardSize, Mat(chessboardCorners), true);
	displayResult("temp", temp);
	waitKey();
	{
		int r = 1;
		for (const auto &P : chessboardCorners) {
			circle(temp, P, (r * 5), Scalar(200, 250, 250), 3);
			++r;
		}
	}

	pair<Point2f, Point2f> twoPoints = getTwoPointsNew(organize(chessboardCorners, boardSize));

	// assume that we could esimate board size in pixel using two leftmost points at the bottom of the chessboard
	Point2f blp = twoPoints.first;
	Point2f blpn = twoPoints.second;
	float squareSize = norm(blp - blpn);
	circle(temp, blp, 30, Scalar(0, 255, 0), 10);
	circle(temp, blpn, 50, Scalar(0, 255, 0), 10);

	vector<Point2f> targetRectangleCorners;
	targetRectangleCorners.push_back(Point2f(blp.x, blp.y)); // bottom left
	targetRectangleCorners.push_back(Point2f(blp.x + squareSize * (boardSize.width - 1), blp.y)); // bottom right
	targetRectangleCorners.push_back(Point2f(blp.x + squareSize * (boardSize.width - 1), blp.y - squareSize * (boardSize.height - 1))); // top right
	targetRectangleCorners.push_back(Point2f(blp.x, blp.y - squareSize * (boardSize.height - 1))); // top left
	cout << "boardSize: " << boardSize << endl;
	cout << "target: " << targetRectangleCorners << endl;

	for (int i = 0; i < 4; ++i) {
		line(temp, targetRectangleCorners[i], targetRectangleCorners[(i + 1) % 4], Scalar(255, 0, 0), 5 + 2 * i);
		circle(temp, targetRectangleCorners[i], 5 * (i + 1), Scalar(255, 255, 0), 5 + 2 * i);
	}

	vector<Point2f> currentRectrangleCorners = extractCornersNew(organize(chessboardCorners, boardSize));
	
	for (int i = 0; i < 4; ++i) {
		line(temp, currentRectrangleCorners[i], currentRectrangleCorners[(i + 1) % 4], Scalar(0, 0, 255), 5 + 2 * i);
		circle(temp, currentRectrangleCorners[i], 10 * (i + 1), Scalar(0, 255, 255), 5 + 2 * i);
	}

	displayResult("temp", temp);
	waitKey();
	imwrite("temp.jpg", temp);

	// find preliminary homography matrix
	Mat preH = findHomography(Mat(currentRectrangleCorners), Mat(targetRectangleCorners), CV_RANSAC);

	vector<Point2f> currentCorners;
	getImageCorners(image, currentCorners);

	Mat tcorners;
	perspectiveTransform(Mat(currentCorners), tcorners, preH);
	vector<Point2f> newCorners = (vector<Point2f>)tcorners;

	corners_info_t ci(newCorners);
	float newWidth = ci.width;
	float newHeight = ci.height;

	// apply offsets
	for (auto &P : targetRectangleCorners) {
		P.x += fabs(ci.minX);
		P.y += fabs(ci.minY);
	}
	for (auto &P : newCorners) {
		P.x += fabs(ci.minX);
		P.y += fabs(ci.minY);
	}

	// recalculate homography accounting offsets
	Mat H = findHomography(Mat(currentRectrangleCorners), Mat(targetRectangleCorners), CV_RANSAC);

	warpPerspective(temp, result, H, Size(newWidth, newHeight));
	corners = newCorners;
	rectangle = targetRectangleCorners;

	return true;
}

int main()
{
	//vector<vector<Point2f>> pointBuf1;
	/*Mat view1 = imread("calib/204.png");

	Mat views[] = {
		imread("calib/3.png"),
		imread("calib/119.png"),
		imread("calib/141.png"),
		imread("calib/204.png"),
		imread("calib/241.png"),
		imread("calib/246.png"),
		imread("calib/313.png"),
		imread("calib/385.png"),
		imread("calib/424.png"),
		imread("calib/452.png"),
		imread("calib/498.png"),
		imread("calib/610.png"),
		imread("calib/645.png"),
		imread("calib/657.png"),
		imread("calib/680.png"),
		imread("calib/722.png"),
	};

	// find chessboard corners to perform calibration
	for (auto &view : views) {
		vector<Point2f> temp;
		bool found1 = findChessboardCorners(view, boardSize, temp,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
		if (found1)
		{
			Mat viewGray;
			cvtColor(view, viewGray, CV_BGR2GRAY);
			cornerSubPix(viewGray, temp, Size(11, 11),
				Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			pointBuf1.push_back(temp);
		}
	}

	// run calibration
	Mat cameraMatrix1;
	Mat distCoeffs1;
	{
		{
			const Size imageSize = view1.size();
			vector<Mat> rvecs, tvecs;
			vector<float> reprojErrors;
			double totalAvgErr = 0;
			if (runCalibration(imageSize, cameraMatrix1, distCoeffs1,  pointBuf1, rvecs, tvecs, reprojErrors, totalAvgErr))
			{
				cout << "Calibration1 succeeded" << ". avg re projection error = " << totalAvgErr << endl;
			}
		}
	}

	Mat uview1;
	Mat uview2;
	Mat left = imread("r182.png");
	Mat right = imread("l182.png");
	imshow("Left", left);
	imshow("Right", right);
	undistort(left, uview1, cameraMatrix1, distCoeffs1);
	undistort(right, uview2, cameraMatrix1, distCoeffs1);*/

	/*vector<Point2f> pointBufLeft;
	vector<Point2f> pointBufRight;

	Mat uview1 = imread("test2a.jpg"), uview2 = imread("test2b.jpg");*/

	vector<Point2f> corners1;
	vector<Point2f> rectangle1;
	Mat result1;
	Mat image1 = imread("IMG_20190207_124236 - Copy.jpg");

	if (!projectToTheFloor(image1, Size(3, 4), result1, rectangle1, corners1)) {
		cout << "Failed to handle first image!" << endl;
		return -1;
	}

	displayResult("image1", image1);
	displayResult("result1", result1);
	waitKey();
	imwrite("result1.jpg", result1);

	vector<Point2f> corners2;
	vector<Point2f> rectangle2;
	Mat result2;
	Mat image2 = imread("IMG_20190207_124244 - Copy.jpg");

	if (!projectToTheFloor(image2, Size(3, 4), result2, rectangle2, corners2)) {
		cout << "Failed to handle second image!" << endl;
		return -1;
	}

	displayResult("image2", image2);
	displayResult("result2", result2);
	waitKey();
	imwrite("result2.jpg", result2);

	vector<Point2f> rotatedRect2;
	if (false) {
		rotatedRect2.push_back(rectangle2[3]);
		rotatedRect2.push_back(rectangle2[0]);
		rotatedRect2.push_back(rectangle2[1]);
		rotatedRect2.push_back(rectangle2[2]);
	} else {
		rotatedRect2 = rectangle2;
	}

	Mat preH = findHomography(Mat(rotatedRect2), Mat(rectangle1), CV_RANSAC);
	vector<Point2f> secondCorners;
	getImageCorners(result2, secondCorners);
	Mat tcorners;
	perspectiveTransform(Mat(secondCorners), tcorners, preH);
	vector<Point2f> newSecondCorners = (vector<Point2f>(tcorners));
	corners_info_t ci(newSecondCorners);

	vector<Point2f> firstCorners;
	getImageCorners(result1, firstCorners);
	corners_info_t fci(firstCorners);

	float minX = fmin(ci.minX, fci.minX);
	float maxX = fmax(ci.maxX, fci.maxX);
	float minY = fmin(ci.minY, fci.minY);
	float maxY = fmax(ci.maxY, fci.maxY);

	float height = maxY - minY;
	float width = maxX - minX;

	Mat result(Size(width, height), image1.type());

	Rect roi1(Point(fabs(minX), fabs(minY)), Size(fci.width, fci.height));
	Rect roi2(Point(0, 0), Size(ci.width, ci.height));

	vector<Point2f> shiftedRect1 = rectangle1;
	for (auto &P : shiftedRect1) {
		P.x += fabs(minX);
		P.y += fabs(minY);
	}

	Mat H = findHomography(Mat(rotatedRect2), Mat(shiftedRect1), CV_RANSAC);

	Mat rotated2;
	warpPerspective(result2, rotated2, H, Size(ci.width, ci.height));

	Mat destRoi1 = result(roi1);
	Mat destRoi2 = result(roi2);

	Mat mask1(Size(fci.width, fci.height), image1.type());
	vector<Point> cm1;
	for (auto &P : corners1) {
		cm1.push_back(P);
	}
	cout << "cm1: " << cm1 << endl;
	fillConvexPoly(mask1, cm1, Scalar(255, 255, 255));

	Mat mask2(Size(ci.width, ci.height), image2.type());
	vector<Point2f> rotatedCorners2;
	if (false) {
		rotatedCorners2.push_back(corners2[3]);
		rotatedCorners2.push_back(corners2[0]);
		rotatedCorners2.push_back(corners2[1]);
		rotatedCorners2.push_back(corners2[2]);
	} else {
		rotatedCorners2 = corners2;
	}

	Mat tcorners2;
	perspectiveTransform(Mat(rotatedCorners2), tcorners2, H);
	vector<Point2f> newCorners2 = (vector<Point2f>)tcorners2;
	vector<Point> cm2;
	for (auto &P : newCorners2) {
		cm2.push_back(P);
	}
	cout << "cm2: " << cm2 << endl;
	fillConvexPoly(mask2, cm2, Scalar(255, 255, 255));
	/*displayResult("mask1", mask1);
	displayResult("mask2", mask2);
	waitKey();*/	

	result1.copyTo(destRoi1, mask1);
	rotated2.copyTo(destRoi2, mask2);

	displayResult("final", result);
	waitKey();
	imwrite("result.jpg", result);



	/*{
		vector<Point2f> t;
		t.push_back(target2[3]);
		t.push_back(target2[0]);
		t.push_back(target2[1]);
		t.push_back(target2[2]);
		cout << "t: " << t << endl;
		Mat H = findHomography(Mat(t), Mat(target1), CV_RANSAC);
		Mat result;
		warpPerspective(result2, result, H, Size(result1.cols * 2, result2.rows * 3));
		imshow("temp", result);

		Mat roi1(result, Rect(0, 0, result1.cols, result1.rows));
		Mat roi2(result, Rect(0, 0, result1.cols, result1.rows));
		result1.copyTo(roi1);
		result.copyTo(roi2);

		Mat rresult;
		resize(result, rresult, Size(result1.cols / 2, result1.rows / 2));
		imshow("final3", rresult);
		waitKey();
	}*/


	return 0;
}

	// find chessboard corners again
	/*{
		bool found1 = findChessboardCorners(uview1, boardSize, pointBufLeft,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
		if (found1)
		{
			Mat viewGray;
			cvtColor(uview1, viewGray, CV_BGR2GRAY);
			cornerSubPix(viewGray, pointBufLeft, Size(11, 11),
				Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
		}

		drawChessboardCorners(uview1, boardSize, Mat(pointBufLeft), found1);

		bool found2 = findChessboardCorners(uview2, boardSize, pointBufRight,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
		if (found2)
		{
			Mat viewGray;
			cvtColor(uview2, viewGray, CV_BGR2GRAY);
			cornerSubPix(viewGray, pointBufRight, Size(11, 11),
				Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
		}

		drawChessboardCorners(uview2, boardSize, Mat(pointBufRight), found2);
	}

	// display intermediate results
	imshow("Left undistort", uview1);
	imshow("Right undistort", uview2);
	waitKey();

	// try to stitch
	{
		Mat H = findHomography(Mat(pointBufRight), Mat(pointBufLeft), CV_RANSAC);
		Mat result;
		warpPerspective(uview2, result, H, Size(uview2.cols * 4, uview2.rows * 2));
		Mat final(Size(uview2.cols * 4 + uview1.cols, uview2.rows * 2), CV_8UC3);
		Mat roi1(final, Rect(0, 0, uview1.cols, uview1.rows));
		Mat roi2(final, Rect(0, 0, result.cols, result.rows));
		result.copyTo(roi2);
		uview1.copyTo(roi1);
		imshow("final1", final);
		waitKey();
		imwrite("final.jpg", final);
	}

    return 0;
}

static double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints,
	const vector<vector<Point2f> >& imagePoints,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs,
	vector<float>& perViewErrors)
{
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); ++i)
	{
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
			distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err * err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

bool runCalibration(const Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,  vector<vector<Point2f>> imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs, vector<float>& reprojErrs, double& totalAvgErr)
{
	cameraMatrix = Mat::eye(3, 3, CV_64F);
	cameraMatrix.at<double>(0, 0) = 1.0;

	distCoeffs = Mat::zeros(8, 1, CV_64F);

	vector<vector<Point3f>	> objectPoints(1);
	for (int i = 0; i < boardSize.height; ++i)
	{
		for (int j = 0; j < boardSize.width; ++j)
		{
			objectPoints[0].push_back(Point3f(float(j*squareSize), float(i*squareSize), 0));
		}
	}

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	//Find intrinsic and extrinsic camera parameters
	double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
		distCoeffs, rvecs, tvecs, CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

	cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
		rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

	return ok;
}*/
