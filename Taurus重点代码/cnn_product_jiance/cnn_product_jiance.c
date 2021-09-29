/*
 * Copyright (c) 2021 HiSilicon (Shanghai) Technologies CO., LIMITED.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "sample_comm_nnie.h"
#include "nnie_sample_plug.h"

#include "hi_ext_util.h"
#include "mpp_help.h"
#include "ai_plug.h"

#define PLUG_UUID          "\"hi.cnn_hand_gesture\""
#define PLUG_DESC          "\"手势识别(cnn)\""     // UTF8 encode

#define FRM_WIDTH          224
#define FRM_HEIGHT         224
// resnet_inst.wk基于开源模型resnet18重训，通过.caffemodel转wk的结果
#define MODEL_FILE_GESTURE    "./plugs/hand_gesture.wk" // 开源模型转换

#define RET_NUM_MAX         4       // 返回number的最大数目trr
#define SCORE_MAX           4096    // 最大概率对应的score
#define THRESH_MIN          30      // 可接受的概率阈值(超过此值则返回给app)

#define TXT_BEGX           20
#define TXT_BEGY           20
#define FONT_WIDTH         32
#define FONT_HEIGHT        40

static OsdSet* g_osdsGesture = NULL;
static HI_S32 g_osd0Gesture = -1;

static const HI_CHAR CNN_HAND_GESTURE[] = "{"
    "\"uuid\": " PLUG_UUID ","
    "\"desc\": " PLUG_DESC ","
    "\"frmWidth\": " HI_TO_STR(FRM_WIDTH) ","
    "\"frmHeight\": " HI_TO_STR(FRM_HEIGHT) ","
    "\"butt\": 0"
"}";

static const HI_CHAR* CnnHandGestureProf(void)
{
    return CNN_HAND_GESTURE;
}

static HI_S32 CnnHandGestureLoad(uintptr_t* model, OsdSet* osds)
{
    SAMPLE_SVP_NNIE_CFG_S *self = NULL;
    HI_S32 ret;

    g_osdsGesture = osds;
    HI_ASSERT(g_osdsGesture);
    g_osd0Gesture = OsdsCreateRgn(g_osdsGesture);
    HI_ASSERT(g_osd0Gesture >= 0);

    ret = CnnCreate(&self, MODEL_FILE_GESTURE);
    *model = ret < 0 ? 0 : (uintptr_t)self;

    return ret;
}

static HI_S32 CnnHandGestureUnload(uintptr_t model)
{
    CnnDestroy((SAMPLE_SVP_NNIE_CFG_S*)model);
    OsdsClear(g_osdsGesture);

    return HI_SUCCESS;
}

/**
    将计算结果打包为resJson.
*/
HI_CHAR* CnnHandGestureToJson(const RecogNumInfo items[], HI_S32 itemNum)
{
    HI_S32 jsonSize = TINY_BUF_SIZE + itemNum * TINY_BUF_SIZE; // 每个item的打包size为TINY_BUF_SIZE
    HI_CHAR *jsonBuf = (HI_CHAR*)malloc(jsonSize);
    HI_ASSERT(jsonBuf);
    HI_S32 offset = 0;

    offset += snprintf_s(jsonBuf + offset, jsonSize - offset, jsonSize - offset - 1, "[");
    for (HI_S32 i = 0; i < itemNum; i++) {
        const RecogNumInfo *item = &items[i];
        uint32_t score = item->score * HI_PER_BASE / SCORE_MAX;
        if (score < THRESH_MIN) {
            break;
        }

        offset += snprintf_s(jsonBuf + offset, jsonSize - offset, jsonSize - offset - 1,
            "%s{ \"classify num\": %u, \"score\": %u }", (i == 0 ? "\n  " : ", "), (uint)item->num, (uint)score);
        HI_ASSERT(offset < jsonSize);
    }
    offset += snprintf_s(jsonBuf + offset, jsonSize - offset, jsonSize - offset - 1, "]");
    HI_ASSERT(offset < jsonSize);
    return jsonBuf;
}

/**
    将计算结果打包为OSD显示内容.
*/
static HI_S32 CnnHandGestureToOsd(const RecogNumInfo items[], HI_S32 itemNum, HI_CHAR* buf, HI_S32 size)
{
    HI_S32 offset = 0;
    HI_CHAR *gesture_name = NULL;

    offset += snprintf_s(buf + offset, size - offset, size - offset - 1, "hand gesture: {");
    for (HI_S32 i = 0; i < itemNum; i++) {
        const RecogNumInfo *item = &items[i];
        uint32_t score = item->score * HI_PER_BASE / SCORE_MAX;
        if (score < THRESH_MIN) {
            break;
        }
        switch (item->num) {
            case 0u:
                gesture_name = "gesture palm";
                break;
            case 1u:
                gesture_name = "gesture first";
                break;
            case 2u:
                gesture_name = "gesture others";
                break;
            default:
                gesture_name = "Unkown";
                break;
        }

        offset += snprintf_s(buf + offset, size - offset, size - offset - 1,
            "%s%s %u:%u%%", (i == 0 ? " " : ", "), gesture_name, (int)item->num, (int)score);
        HI_ASSERT(offset < size);
    }
    offset += snprintf_s(buf + offset, size - offset, size - offset - 1, " }");
    HI_ASSERT(offset < size);
    return offset;
}

static HI_S32 CnnHandGestureCal(uintptr_t model,
    VIDEO_FRAME_INFO_S *srcFrm, VIDEO_FRAME_INFO_S *resFrm, HI_CHAR** resJson)
{
    SAMPLE_SVP_NNIE_CFG_S *self = (SAMPLE_SVP_NNIE_CFG_S*)model; // reference to SDK sample_comm_nnie.h Line 99
    IVE_IMAGE_S img; // referece to SDK hi_comm_ive.h Line 143
    static HI_CHAR prevOsd[NORM_BUF_SIZE] = ""; // 安全，插件架构约定同时只会有一个线程访问插件
    HI_CHAR osdBuf[NORM_BUF_SIZE] = "";
    /*
        01-palm          02_first
        03_others
    */
    RecogNumInfo resBuf[RET_NUM_MAX] = {0};
    HI_S32 reslen = 0;
    HI_S32 ret;

    ret = FrmToOrigImg((VIDEO_FRAME_INFO_S*)srcFrm, &img);
    HI_EXP_RET(ret != HI_SUCCESS, ret, "CnnTrashClassifyCal FAIL, for YUV2RGB FAIL, ret=%#x\n", ret);

    ret = CnnCalU8c1Img(self, &img, resBuf, HI_ARRAY_SIZE(resBuf), &reslen); // 沿用该推理逻辑
    HI_EXP_LOGE(ret < 0, "cnn cal FAIL, ret=%d\n", ret);
    HI_ASSERT(reslen <= sizeof(resBuf) / sizeof(resBuf[0]));

    // 生成resJson和resOsd
    HI_CHAR *jsonBuf = CnnHandGestureToJson(resBuf, reslen);
    *resJson = jsonBuf;
    CnnHandGestureToOsd(resBuf, reslen, osdBuf, sizeof(osdBuf));

    // 仅当resJson与此前计算发生变化时,才重新打OSD输出文字
    if (strcmp(osdBuf, prevOsd) != 0) {
        HiStrxfrm(prevOsd, osdBuf, sizeof(prevOsd));

        // 叠加图形到resFrm中
        HI_OSD_ATTR_S rgn;
        TxtRgnInit(&rgn, osdBuf, TXT_BEGX, TXT_BEGY, ARGB1555_YELLOW2, FONT_WIDTH, FONT_HEIGHT);
        OsdsSetRgn(g_osdsGesture, g_osd0Gesture, &rgn);
        LOGI("CNN hand gesture: %s\n", osdBuf);
    }
    return ret;
}

static const AiPlug G_HAND_GESTURE_ITF = {
    .Prof = CnnHandGestureProf,
    .Load = CnnHandGestureLoad,
    .Unload = CnnHandGestureUnload,
    .Cal = CnnHandGestureCal,
};

const AiPlug* AiPlugItf(uint32_t* magic)
{
    if (magic) {
        *magic = AI_PLUG_MAGIC;
    }

    return (AiPlug*)&G_HAND_GESTURE_ITF;
}
