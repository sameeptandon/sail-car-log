#include "LanePredictor.h"

class LanePredictor_honda : public LanePredictor{
    public:
        LanePredictor_honda();
        LanePredictor_honda(int* argc, char** argv, int stream_num);
        ~LanePredictor_honda();
        Ptr<ArrayViewHandle> processImage(const Ptr<ArrayViewHandle>& img);

    private:
        Ptr<ArrayViewHandle> mean_lcn;
        Ptr<ArrayViewHandle> mean_lcn_sqr;
        Ptr<ArrayViewHandle> divide_lcn;
        Ptr<ArrayViewHandle> filt_LCN;
        Ptr<ArrayViewHandle> W_1;
        Ptr<ArrayViewHandle> filt_1;
        Ptr<ArrayViewHandle> nonlin_1;
        Ptr<ArrayViewHandle> pool_1;
        Ptr<ArrayViewHandle> W_2;
        Ptr<ArrayViewHandle> filt_2;
        Ptr<ArrayViewHandle> nonlin_2;
        Ptr<ArrayViewHandle> pool_2;
        Ptr<ArrayViewHandle> W_3;
        Ptr<ArrayViewHandle> feat;
        Ptr<ArrayViewHandle> W_bn;
        Ptr<ArrayViewHandle> B_bn;
        Ptr<ArrayViewHandle> feat_bn;
        Ptr<ArrayViewHandle> W_final;
        Ptr<ArrayViewHandle> B_final;
        Ptr<ArrayViewHandle> mult_linear;
        Ptr<ArrayViewHandle> reduce_col;
        Ptr<ArrayViewHandle> indexArray;
        Ptr<ArrayViewHandle> tmpArray;
        Ptr<ArrayViewHandle> labels;
        Ptr<ArrayViewHandle> data_buf_scratch;
        Ptr<ArrayViewHandle> mean_lcn_scratch;
        Ptr<ArrayViewHandle> mean_lcn_sqr_scratch;
        Ptr<ArrayViewHandle> divide_lcn_scratch;
        Ptr<ArrayViewHandle> filt_1_scratch;
        Ptr<ArrayViewHandle> divide_lcn_in_scratch;
        Ptr<ArrayViewHandle> pool_1_scratch;
        Ptr<ArrayViewHandle> filt_2_scratch;
        Ptr<ArrayViewHandle> pool_2_scratch;
        Ptr<ArrayViewHandle> feat_scratch;
};
