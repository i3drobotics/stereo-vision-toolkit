#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <vector>
#include <string>

#ifndef IMAGE2STRING_H_
#define IMAGE2STRING_H_

/**
 * Classe que converte as imagens para base64 e virse e versa
 */
class Image2String {
public:
	/**
	 * Constritor default da classe
	 */
    Image2String();
	
	/**
	 * Método que converte uma imagem base64 em um cv::Mat
	 * @param imageBase64, imagem em base64
	 * @return imagem em cv::Mat
	 */
    static cv::Mat str2mat(const std::string& imageBase64);
	
	/**
	 * Método que converte uma cv::Mat numa imagem em base64
	 * @param img, imagem em cv::Mat
	 * @return imagem em base64
	 */
    static std::string ucharMat2str(const cv::Mat& img, int quality);

    static std::string floatMat2str(const cv::Mat& img, int quality);

    virtual ~Image2String();

private:
    static std::string base64_encode(uchar const* bytesToEncode, unsigned int inLen);

    static std::string base64_decode(std::string const& encodedString);

};

#endif /* CONVERTIMAGE_H_ */
