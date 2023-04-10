// 词袋模型的图像相识度计算
use crate::bag_opecv_util::{all_kps_from_dir, load_img_get_kps};
use abow::vocab::Vocabulary;
use abow::Desc;
use abow::{BoW, DirectIdx};

struct ImgSimilarity {
    // 保存训练好的词袋模型
    voc: Vocabulary,
}

impl ImgSimilarity {
    // 从文件中加载词袋模型
    pub fn load_from_file<P: AsRef<std::path::Path>>(path: P) -> Self {
        let features = all_kps_from_dir(path.as_ref().to_str().unwrap()).unwrap();
        let voc = Vocabulary::create(&features, 10, 5);
        Self { voc }
    }

    // 获取图像的特征向量
    pub fn get_feature_vector<P: AsRef<std::path::Path>>(&self, path: P) -> (BoW, DirectIdx) {
        let new_feat = load_img_get_kps(path.as_ref().to_str().unwrap()).unwrap();
        self.voc.transform_with_direct_idx(&new_feat).unwrap()
    }

    // 计算相似度
    pub fn similarity(&self, a: &BoW, b: &BoW) -> f32 {
        a.l1(b)
    }
}

mod tests {
    // 添加测试
    #[cfg(test)]
    fn test_similarity() {
        use super::*;
        let sim = ImgSimilarity::load_from_file("data/train");
        let (a, _) = sim.get_feature_vector("data/test/1.jpg");
        let (b, _) = sim.get_feature_vector("data/test/2.jpg");
        println!("value: {} ", sim.similarity(&a, &b));
    }
}
