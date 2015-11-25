#ifndef SFM_H
#define SFM_H

#include <opencv2\opencv.hpp>
//#include <opencv2\nonfree\nonfree.hpp> // SIFTまたはSURFを使う場合は必要
//#include <opencv2\xfeatures2d\nonfree.hpp>

class SfM
{
public:
	//カメラ
	WebCamera camera;
	//プロジェクタ
	WebCamera projector;

	//基礎行列
	cv::Mat F;
	//基本行列
	cv::Mat E;

	//使用する画像
	cv::Mat src_camImage; // 画像1のファイル名
	cv::Mat src_projImage; // 画像2のファイル名
	cv::Mat result; //結果描画用

	//特徴点から抽出した対応点
	std::vector<cv::Point2f>cam_pts, proj_pts;
	
	//コンストラクタ
	SfM(const char *camImageName, const char *projImageName, WebCamera cam, WebCamera proj)
	{
		camera = cam;
		projector = proj;

		std::cout << "cam_K:\n" << camera.cam_K << std::endl;
		std::cout << "cam_dist:\n" << camera.cam_dist << std::endl;
		std::cout << "proj_K:\n" << projector.cam_K << std::endl;
		std::cout << "proj_dist:\n" << projector.cam_dist << std::endl;

		//歪み除去して読み込み(1枚目：カメラ　2枚目:プロジェクタ)
		cv::undistort(cv::imread(camImageName), src_camImage, camera.cam_K, camera.cam_dist);
		cv::undistort(cv::imread(projImageName), src_projImage, projector.cam_K, projector.cam_dist);
	};
	~SfM(){};

	void featureMatching(	const char *featureDetectorName, const char *descriptorExtractorName, const char *descriptorMatcherName, bool crossCheck)
	{
/*
		if(featureDetectorName == "SIFT" || featureDetectorName == "SURF" 
			|| descriptorExtractorName == "SIFT" || descriptorExtractorName == "SURF")
		{
			// SIFTまたはSURFを使う場合はこれを呼び出す．
			cv::initModule_nonfree();
		}
*/
		// 特徴点抽出
		cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create(featureDetectorName);
		std::vector<cv::KeyPoint> keypoint1, keypoint2;//1->camera 2->projector
		detector->detect(src_camImage, keypoint1);
		detector->detect(src_projImage, keypoint2);

		// 特徴記述
		cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create(descriptorExtractorName);
		cv::Mat descriptor1, descriptor2;
		extractor->compute(src_camImage, keypoint1, descriptor1);
		extractor->compute(src_projImage, keypoint2, descriptor2);

		// マッチング
		cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(descriptorMatcherName);
		std::vector<cv::DMatch> dmatch;
		if (crossCheck)
		{
			// クロスチェックする場合
			std::vector<cv::DMatch> match12, match21;
			matcher->match(descriptor1, descriptor2, match12);
			matcher->match(descriptor2, descriptor1, match21);
			for (size_t i = 0; i < match12.size(); i++)
			{
				cv::DMatch forward = match12[i];
				cv::DMatch backward = match21[forward.trainIdx];
				if (backward.trainIdx == forward.queryIdx)
					dmatch.push_back(forward);
			}
		}
		else
		{
			// クロスチェックしない場合
			matcher->match(descriptor1, descriptor2, dmatch);
		}

		//最小距離
		double min_dist = DBL_MAX;
		for(int j = 0; j < (int)dmatch.size(); j++)
		{
			double dist = dmatch[j].distance;
			if(dist < min_dist) min_dist = (dist < 1.0) ? 1.0 : dist;
		}

		//良いペアのみ残す
		double cutoff = 5.0 * min_dist;
		std::set<int> existing_trainIdx;
		std::vector<cv::DMatch> matches_good;
		for(int j = 0; j < (int)dmatch.size(); j++)
		{
			if(dmatch[j].trainIdx <= 0) dmatch[j].trainIdx = dmatch[j].imgIdx;
//			if(dmatch[j].distance > 0.0 && dmatch[j].distance < cutoff){
			//x座標で決め打ちしきい値(マスクの代わり)
			if(dmatch[j].distance > 0.0 && dmatch[j].distance < cutoff && keypoint1[dmatch[j].queryIdx].pt.x > 240 && keypoint2[dmatch[j].trainIdx].pt.x > 240){
				if(existing_trainIdx.find(dmatch[j].trainIdx) == existing_trainIdx.end() && dmatch[j].trainIdx >= 0 && dmatch[j].trainIdx < (int)keypoint2.size()) {
					matches_good.push_back(dmatch[j]);
                    existing_trainIdx.insert(dmatch[j].trainIdx);
				}
			}
		}

        // 対応点の登録(5ペア以上は必要)
        if (matches_good.size() > 10) {
            for (int j = 0; j < (int)matches_good.size(); j++) {
                cam_pts.push_back(keypoint1[matches_good[j].queryIdx].pt);
                proj_pts.push_back(keypoint2[matches_good[j].trainIdx].pt);
            }
		}

		// マッチング結果の表示
		cv::drawMatches(src_camImage, keypoint1, src_projImage, keypoint2, matches_good, result);
		//cv::drawMatches(src_image1, keypoint1, src_image2, keypoint2, dmatch, result);
		cv::imshow("good matching", result);
		cv::waitKey(0);
	}


	cv::Mat findEssientialMat(){
		//基礎行列の算出
		//findfundamentalMat( pt1, pt2, F行列を計算する手法, 点からエピポーラ線までの最大距離, Fの信頼度)
		if(cam_pts.size() == 7)
			F = cv::findFundamentalMat(cam_pts, proj_pts,cv::FM_7POINT, 3.0, 0.99);
		else if(cam_pts.size() == 8)
			F = cv::findFundamentalMat(cam_pts, proj_pts,cv::FM_8POINT, 3.0, 0.99);
		else
			F = cv::findFundamentalMat(cam_pts, proj_pts,cv::RANSAC, 3.0, 0.99);

		//基本行列の算出
		E = camera.cam_K.t() * F * projector.cam_K;


		return E;
	}


	void findProCamPose(const cv::Mat& E, const cv::Mat& R, const cv::Mat& t)
	{
/*			// 焦点距離とレンズ主点
            double fovx, fovy, focal, pasp;
            cv::Point2d pp;
			cv::calibrationMatrixValues(cam_K, cv::Size(camera.width, camera.height), 0.0, 0.0, fovx, fovy, focal, pp, pasp);
 
            // 5点アルゴリズムで基礎行列を計算
			cv::Matx33d E = cv::findEssentialMat(cam_pts, proj_pts, focal, pp);
 */
            cv::Matx33d R1, R2;
            cv::Matx31d t_;

			//[R1,t] [R1, -t] [R2, t], [R2, -t]の可能性がある
			cv::decomposeEssentialMat(E, R1, R2, t_);
			std::cout << "\nR1:\n" << R1 << std::endl;
			std::cout << "R2:\n" << R2 << std::endl;
			std::cout << "t:\n" << t_ << std::endl;
	}
};
	
#endif