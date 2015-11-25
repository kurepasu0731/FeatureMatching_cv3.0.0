#ifndef FEATUREMATCHING_H
#define FEATUREMATCHING_H

#include <opencv2\opencv.hpp>

class FeatureMatching
{
public:
	cv::Mat src_image1; // �摜1�̃t�@�C����
	cv::Mat src_image2; // �摜2�̃t�@�C����
	cv::Mat result; //�}�b�`���O����

	std::string featureDetectorName; // detectorType
	std::string descriptorExtractorName; // descriptorExtractorType
	std::string descriptorMatcherName; // descriptorMatcherType
	bool crossCheck; // �}�b�`���O���ʂ��N���X�`�F�b�N���邩�ǂ���

	//�R���X�g���N�^
	FeatureMatching(const char *image1Name, const char *image2Name, 
		const char *_featureDetectorName, const char *_descriptorExtractorName, const char *_descriptorMatcherName, bool _crossCheck)
	{
		src_image1 = cv::imread(image1Name);
		src_image2 = cv::imread(image2Name);
		featureDetectorName = _featureDetectorName;
		descriptorExtractorName = _descriptorExtractorName;
		descriptorMatcherName = _descriptorMatcherName;
		crossCheck = _crossCheck;
	};

	~FeatureMatching(){};

	//�����_�}�b�`���O���s
	void apply()
	{
		if(featureDetectorName == "SIFT" || featureDetectorName == "SURF" 
			|| descriptorExtractorName == "SIFT" || descriptorExtractorName == "SURF")
		{
			// SIFT�܂���SURF���g���ꍇ�͂�����Ăяo���D
			cv::initModule_nonfree();
		}

		// �����_���o
		cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create(featureDetectorName);
		std::vector<cv::KeyPoint> keypoint1, keypoint2;
		detector->detect(src_image1, keypoint1);
		detector->detect(src_image2, keypoint2);

		// �����L�q
		cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create(descriptorExtractorName);
		cv::Mat descriptor1, descriptor2;
		extractor->compute(src_image1, keypoint1, descriptor1);
		extractor->compute(src_image2, keypoint2, descriptor2);

		// �}�b�`���O
		cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(descriptorMatcherName);
		std::vector<cv::DMatch> dmatch;
		if (crossCheck)
		{
			// �N���X�`�F�b�N����ꍇ
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
			// �N���X�`�F�b�N���Ȃ��ꍇ
			matcher->match(descriptor1, descriptor2, dmatch);
		}

		// �}�b�`���O���ʂ̕\��
		cv::drawMatches(src_image1, keypoint1, src_image2, keypoint2, dmatch, result);
		cv::imshow("matching", result);
	}

	void saveResult(const char *resultImageName)
	{
		cv::imwrite(resultImageName, result);
	}
};

#endif