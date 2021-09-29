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
#include "audio_test.h"
#include "hisignalling.h"

#define PLUG_UUID          "\"hi.cnn_trash_classify\""
#define PLUG_DESC          "\"垃圾分类(cnn)\""     // UTF8 encode

#define FRM_WIDTH          224
#define FRM_HEIGHT         224
// resnet_inst.wk基于开源模型resnet18重训，通过.caffemodel转wk的结果
#define MODEL_FILE_TRASH    "./plugs/resnet_inst.wk" // 开源模型转换

#define RET_NUM_MAX         4 		// 返回number的最大数目trr
#define SCORE_MAX           4096 	// 最大概率对应的score
#define THRESH_MIN          30 		// 可接受的概率阈值(超过此值则返回给app)

#define TXT_BEGX           20
#define TXT_BEGY           20
#define FONT_WIDTH         32
#define FONT_HEIGHT        40
#define AUDIO_CASE_TWO     2
#define AUDIO_SCORE        90 		// 置信度可自行配置
#define AUDIO_FRAME        14 		// 每隔15帧识别一次，可自行配置

static OsdSet* g_osdsTrash = NULL;
static HI_S32 g_osd0Trash = -1;
static int g_num = 108;
static int g_count = 0;
static pthread_t g_thrdId = 0;
static int g_supportAudio = 0;

static SkPair g_stmChn = {
    .in = -1,
    .out = -1
};

static void PlayAudio(const RecogNumInfo items)
{
	if (g_count < AUDIO_FRAME) {
		g_count++;
		return;
	}

	const RecogNumInfo *item = &items;
	uint32_t score = item->score * 100 / 4096;
	if ((score > AUDIO_SCORE) && (g_num != item->num)) {
		g_num = item->num;
		audio_test(AUDIO_CASE_TWO, g_num, -1);
	}
	g_count = 0;
}

static void* GetAudioFileName(void* arg)
{
	RecogNumInfo resBuf = {0};
	int ret;
	
	while(1) {
		ret = FdReadMsg(g_stmChn.in, &resBuf, sizeof(RecogNumInfo));
		if (ret == sizeof(RecogNumInfo)) {
			PlayAudio(resBuf);
		}
	}
}

static const HI_CHAR CNN_TRASH_CLASSIFY[] = "{"
    "\"uuid\": " PLUG_UUID ","
    "\"desc\": " PLUG_DESC ","
    "\"frmWidth\": " HI_TO_STR(FRM_WIDTH) ","
    "\"frmHeight\": " HI_TO_STR(FRM_HEIGHT) ","
    "\"butt\": 0"
"}";

static const HI_CHAR* CnnTrashClassifyProf(void)
{
    return CNN_TRASH_CLASSIFY;
}

static int uart_fd = -1;
static HI_S32 CnnTrashClassifyLoad(uintptr_t* model, OsdSet* osds)
{
    SAMPLE_SVP_NNIE_CFG_S *self = NULL;
    HI_S32 ret;

    g_osdsTrash = osds;
    HI_ASSERT(g_osdsTrash);
    g_osd0Trash = OsdsCreateRgn(g_osdsTrash);
    HI_ASSERT(g_osd0Trash >= 0);

    ret = CnnCreate(&self, MODEL_FILE_TRASH);
    *model = ret < 0 ? 0 : (uintptr_t)self;

	if (GetCfgBool("audio_player:support_audio", true)) {
		ret = SkPairCreate(&g_stmChn);
		HI_ASSERT(ret == 0);
		if (pthread_create(&g_thrdId, NULL, GetAudioFileName, NULL) < 0) {
			HI_ASSERT(0);
		}
		g_supportAudio = 1;
	}
	
	uart_fd = uartOpenInit();
	HI_ASSERT(uart_fd >= 0);
	
	return ret;
}

static HI_S32 CnnTrashClassifyUnload(uintptr_t model)
{
    CnnDestroy((SAMPLE_SVP_NNIE_CFG_S*)model);
    OsdsClear(g_osdsTrash);
	if (g_supportAudio == 1) {
		SkPairDestroy(&g_stmChn);
		pthread_join(g_thrdId, NULL);
	}
	
	close(uart_fd);
	
    return HI_SUCCESS;
}

/**
    将计算结果打包为resJson.
*/
HI_CHAR* CnnTrashClassifyToJson(const RecogNumInfo items[], HI_S32 itemNum)
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
static HI_S32 CnnTrashClassifyToOsd(const RecogNumInfo items[], HI_S32 itemNum, HI_CHAR* buf, HI_S32 size)
{
    HI_S32 offset = 0;
    HI_CHAR *trash_name = NULL;

    offset += snprintf_s(buf + offset, size - offset, size - offset - 1, "trash classify: {");
    for (HI_S32 i = 0; i < itemNum; i++) {
        const RecogNumInfo *item = &items[i];
        uint32_t score = item->score * HI_PER_BASE / SCORE_MAX;
        if (score < THRESH_MIN) {
            break;
        }
        switch (item->num) {
            case 0u:
/*             case 1u:
            case 2u:
            case 3u:
            case 4u:
            case 5u: */
                trash_name = "ONE_0.8MM";
				usbUartSendRead(uart_fd,FIST);
				usleep(100*100);
                break;
            case 1u:
/*             case 7u:
            case 8u:
            case 9u: */
                trash_name = "qualified";
				usbUartSendRead(uart_fd,PALM);
				usleep(100*100);
                break;
            case 2u:
/*             case 11u:
            case 12u:
            case 13u:
            case 14u:
            case 15u: */
                trash_name = "ONE_2.0MM";
				usbUartSendRead(uart_fd,X_BIG);
				usleep(100*100);
                break;
            case 3u:
/*             case 17u:
            case 18u:
            case 19u: */
                trash_name = "TWO";
				usbUartSendRead(uart_fd,X_LIT);
				usleep(100*100);
                break;
            default:
                trash_name = "unqualified";
				usbUartSendRead(uart_fd,Y_BIG);
                break;
        }

        offset += snprintf_s(buf + offset, size - offset, size - offset - 1,
            "%s%s %u:%u%%", (i == 0 ? " " : ", "), trash_name, (int)item->num, (int)score);
        HI_ASSERT(offset < size);
    }
    offset += snprintf_s(buf + offset, size - offset, size - offset - 1, " }");
    HI_ASSERT(offset < size);
    return offset;
}

static HI_S32 CnnTrashClassifyCal(uintptr_t model,
    VIDEO_FRAME_INFO_S *srcFrm, VIDEO_FRAME_INFO_S *resFrm, HI_CHAR** resJson)
{
    SAMPLE_SVP_NNIE_CFG_S *self = (SAMPLE_SVP_NNIE_CFG_S*)model; // reference to SDK sample_comm_nnie.h Line 99
    IVE_IMAGE_S img; // referece to SDK hi_comm_ive.h Line 143
    static HI_CHAR prevOsd[NORM_BUF_SIZE] = ""; // 安全，插件架构约定同时只会有一个线程访问插件
    HI_CHAR osdBuf[NORM_BUF_SIZE] = "";
    /*
        01-Kitchen_Watermelon_rind    02_Kitchen_Egg_shell
        03_Kitchen_Fishbone           04_Kitchen_Eggplant
        05_Kitchen_Scallion           06_Kitchen_Mushromm
        07_Hazardous_Waste_battery    08_Hazardous_Expired_cosmetrics
        09_Hazardous_Woundplast       10_Hazardous_Medical_gauze
        11_Recyclabel_Old_dolls       12_Recyclabel_Old_clip
        13_Recyclabel_Toothbrush      14_Recyclabel_Milk_box
        15_Recyclabel_Old_handbag     16_Recyclabel_Zip_top_can
        17_other_Ciggrate_end         18_Other_Bad_closestool
        19_other_Brick                20_Other_Dish
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
    HI_CHAR *jsonBuf = CnnTrashClassifyToJson(resBuf, reslen);
    *resJson = jsonBuf;
    CnnTrashClassifyToOsd(resBuf, reslen, osdBuf, sizeof(osdBuf));
	if (g_supportAudio == 1) {
		if (FdWriteMsg(g_stmChn.out, &resBuf[0], sizeof(RecogNumInfo)) != sizeof(RecogNumInfo)) {
			LOGE("FdWriteMsg FAIL\n"); 
		}
	}
	
    // 仅当resJson与此前计算发生变化时,才重新打OSD输出文字
    if (strcmp(osdBuf, prevOsd) != 0) {
        HiStrxfrm(prevOsd, osdBuf, sizeof(prevOsd));

        // 叠加图形到resFrm中
        HI_OSD_ATTR_S rgn;
        TxtRgnInit(&rgn, osdBuf, TXT_BEGX, TXT_BEGY, ARGB1555_YELLOW2, FONT_WIDTH, FONT_HEIGHT);
        OsdsSetRgn(g_osdsTrash, g_osd0Trash, &rgn);
        LOGI("CNN trash classify: %s\n", osdBuf);
    }
    return ret;
}

static const AiPlug G_CNN_TRASH_CLASSIFY_ITF = {
    .Prof = CnnTrashClassifyProf,
    .Load = CnnTrashClassifyLoad,
    .Unload = CnnTrashClassifyUnload,
    .Cal = CnnTrashClassifyCal,
};

const AiPlug* AiPlugItf(uint32_t* magic)
{
    if (magic) {
        *magic = AI_PLUG_MAGIC;
    }

    return (AiPlug*)&G_CNN_TRASH_CLASSIFY_ITF;
}
