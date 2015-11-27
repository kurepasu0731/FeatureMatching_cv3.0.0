#ifndef SFM_H
#define SFM_H

#include "WebCamera.h"
#include <opencv2\opencv.hpp>
#include <opencv2\xfeatures2d\nonfree.hpp>
#include <set>
//#include <opencv2\nonfree\nonfree.hpp> // SIFTまたはSURFを使う場合は必要

class SfM
{
public:
	//カメラ
	WebCamera camera;
	//プロジェクタ
	WebCamera projector;

	//基礎行列
	//cv::Mat F;
	//基本行列
	//cv::Mat E;

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

		//歪み補正なし
		//src_camImage = cv::imread(camImageName);
		//src_projImage = cv::imread(projImageName);
	};
	~SfM(){};

	void featureMatching(	const char *featureDetectorName, const char *descriptorExtractorName, const char *descriptorMatcherName, bool crossCheck)
	{
		if(featureDetectorName == "SIFT" || featureDetectorName == "SURF" 
			|| descriptorExtractorName == "SIFT" || descriptorExtractorName == "SURF")
		{
			// SIFTまたはSURFを使う場合はこれを呼び出す．
			//cv::initModule_nonfree();
			cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create();
		}

		// 特徴点抽出
		//cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create(featureDetectorName);
		cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
		std::vector<cv::KeyPoint> keypoint1, keypoint2;//1->camera 2->projector
		detector->detect(src_camImage, keypoint1);
		detector->detect(src_projImage, keypoint2);

		//// 特徴記述
		//cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create(descriptorExtractorName);
		cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create();
		cv::Mat descriptor1, descriptor2;
		extractor->compute(src_camImage, keypoint1, descriptor1);
		extractor->compute(src_projImage, keypoint2, descriptor2);

		// マッチング
		cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(descriptorMatcherName);
		//cv::Ptr<cv::DescriptorMatcher> matcher = cv::FlannBasedMatcher::create(descriptorMatcherName);
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

//		//最小距離
//		double min_dist = DBL_MAX;
//		for(int j = 0; j < (int)dmatch.size(); j++)
//		{
//			double dist = dmatch[j].distance;
//			if(dist < min_dist) min_dist = (dist < 1.0) ? 1.0 : dist;
//		}
//
//		//良いペアのみ残す
//		double cutoff = 5.0 * min_dist;
//		std::set<int> existing_trainIdx;
//		std::vector<cv::DMatch> matches_good;
//		for(int j = 0; j < (int)dmatch.size(); j++)
//		{
//			if(dmatch[j].trainIdx <= 0) dmatch[j].trainIdx = dmatch[j].imgIdx;
//			if(dmatch[j].distance > 0.0 && dmatch[j].distance < cutoff){
//			//x座標で決め打ちしきい値(マスクの代わり)
////			if(dmatch[j].distance > 0.0 && dmatch[j].distance < cutoff && keypoint1[dmatch[j].queryIdx].pt.x > 240 && keypoint2[dmatch[j].trainIdx].pt.x > 240){
//				if(existing_trainIdx.find(dmatch[j].trainIdx) == existing_trainIdx.end() && dmatch[j].trainIdx >= 0 && dmatch[j].trainIdx < (int)keypoint2.size()) {
//					matches_good.push_back(dmatch[j]);
//                    existing_trainIdx.insert(dmatch[j].trainIdx);
//				}
//			}
//		}
//
//        // 対応点の登録(5ペア以上は必要)
//        if (matches_good.size() > 10) {
//            for (int j = 0; j < (int)matches_good.size(); j++) {
//                cam_pts.push_back(keypoint1[matches_good[j].queryIdx].pt);
//                proj_pts.push_back(keypoint2[matches_good[j].trainIdx].pt);
//            }
//		}
        if (dmatch.size() > 10) {
            for (int j = 0; j < (int)dmatch.size(); j++) {
                cam_pts.push_back(keypoint1[dmatch[j].queryIdx].pt);
                proj_pts.push_back(keypoint2[dmatch[j].trainIdx].pt);
            }
		}

		// マッチング結果の表示
		//cv::drawMatches(src_camImage, keypoint1, src_projImage, keypoint2, matches_good, result);
		cv::drawMatches(src_camImage, keypoint1, src_projImage, keypoint2, dmatch, result);
		//cv::Mat resize;
		//result.copyTo(resize);
		//cv::resize(result, resize, resize.size(), 0.5, 0.5);
		cv::imshow("good matching", result);
		cv::waitKey(0);
	}

	void saveResult(const char *resultImageName)
	{
		cv::imwrite(resultImageName, result);
	}

	cv::Mat findEssientialMat(){
		// 焦点距離とレンズ主点
        double cam_f, proj_f, cam_fovx, cam_fovy, proj_fovx, proj_fovy, cam_pasp, proj_pasp;
        cv::Point2d cam_pp, proj_pp;
		cv::calibrationMatrixValues(camera.cam_K, cv::Size(camera.width, camera.height), 0.0, 0.0, cam_fovx, cam_fovy, cam_f, cam_pp, cam_pasp);
		cv::calibrationMatrixValues(projector.cam_K, cv::Size(projector.width, projector.height), 0.0, 0.0, proj_fovx, proj_fovy, proj_f, proj_pp, proj_pasp);

		return cv::findEssentialMat(cam_pts, proj_pts, cam_f, cam_pp, CV_RANSAC, 0.1, 0.99);

	}


	void findProCamPose(const cv::Mat& E, const cv::Mat& R, const cv::Mat& t)
	{
            cv::Mat R1 = cv::Mat::eye(3,3,CV_64F);
            cv::Mat R2 = cv::Mat::eye(3,3,CV_64F);
			cv::Mat t_ = cv::Mat::zeros(3,1,CV_64F);
			//[R1,t] [R1, -t] [R2, t], [R2, -t]の可能性がある
			decomposeEssentialMat(E, R1, R2, t_);

			std::cout << "\nR1:\n" << R1 << std::endl;
			std::cout << "R2:\n" << R2 << std::endl;
			std::cout << "t:\n" << t_ << std::endl;
	}

	void recoverPose( cv::InputArray E, cv::OutputArray _R, cv::OutputArray _t)
	{
		////カメラ
		//double cam_fx = camera.cam_K.at<double>(0,0);
		//double cam_fy = camera.cam_K.at<double>(1,1);
		//cv::Point2d cam_pp = cv::Point2d(camera.cam_K.at<double>(0,2), camera.cam_K.at<double>(1,2));
		////プロジェクタ
		//double proj_fx = projector.cam_K.at<double>(0,0);
		//double proj_fy = projector.cam_K.at<double>(1,1);
		//cv::Point2d proj_pp = cv::Point2d(projector.cam_K.at<double>(0,2), projector.cam_K.at<double>(1,2));

		// 焦点距離とレンズ主点
        double cam_f, proj_f, cam_fovx, cam_fovy, proj_fovx, proj_fovy, cam_pasp, proj_pasp;
        cv::Point2d cam_pp, proj_pp;
		cv::calibrationMatrixValues(camera.cam_K, cv::Size(camera.width, camera.height), 0.0, 0.0, cam_fovx, cam_fovy, cam_f, cam_pp, cam_pasp);
		cv::calibrationMatrixValues(projector.cam_K, cv::Size(projector.width, projector.height), 0.0, 0.0, proj_fovx, proj_fovy, proj_f, proj_pp, proj_pasp);

		cv::recoverPose(E, cam_pts, proj_pts, _R, _t, cam_f, cam_pp);

	}
};
	
#endif