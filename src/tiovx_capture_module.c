/*
 *
 * Copyright (c) 2023 Texas Instruments Incorporated
 *
 * All rights reserved not granted herein.
 *
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive
 * license under copyrights and patents it now or hereafter owns or controls to make,
 * have made, use, import, offer to sell and sell ("Utilize") this software subject to the
 * terms herein.  With respect to the foregoing patent license, such license is granted
 * solely to the extent that any such patent is necessary to Utilize the software alone.
 * The patent license shall not apply to any combinations which include this software,
 * other than combinations with devices manufactured by or for TI ("TI Devices").
 * No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce this license
 * (including the above copyright notice and the disclaimer and (if applicable) source
 * code license limitations below) in the documentation and/or other materials provided
 * with the distribution
 *
 * Redistribution and use in binary form, without modification, are permitted provided
 * that the following conditions are met:
 *
 * *       No reverse engineering, decompilation, or disassembly of this software is
 * permitted with respect to any software provided in binary form.
 *
 * *       any redistribution and use are licensed by TI for use only with TI Devices.
 *
 * *       Nothing shall obligate TI to provide you with source code for the software
 * licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution of the
 * source code are permitted provided that the following conditions are met:
 *
 * *       any redistribution and use of the source code, including any resulting derivative
 * works, are licensed by TI for use only with TI Devices.
 *
 * *       any redistribution and use of any object code compiled from the source code
 * and any resulting derivative works, are licensed by TI for use only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers
 *
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <tiovx_capture_module.h>
#include <iss_sensors.h>
#include <iss_sensor_if.h>

static vx_status tiovx_capture_module_create_output(vx_context context, TIOVXCaptureModuleObj *captureObj, SensorObj *sensorObj)
{
    vx_status status = VX_SUCCESS;

    IssSensor_CreateParams *sensorParams = &sensorObj->sensorParams;
    vx_int32 buf;

    /* RAW format */
    if(0 == captureObj->capture_format)
    {
        tivx_raw_image raw_image = tivxCreateRawImage(context, &sensorParams->sensorInfo.raw_params);
        status = vxGetStatus((vx_reference)raw_image);

        if(status == VX_SUCCESS)
        {
            for(buf = 0; buf < captureObj->out_bufq_depth; buf++)
            {
                /* 
                    Creating an Object array of size equal to num_cameras_enabled.
                    Each member of object array will be an raw image from a camera
                */
                captureObj->image_arr[buf] = vxCreateObjectArray(context, (vx_reference)raw_image, sensorObj->num_cameras_enabled);
                status = vxGetStatus((vx_reference)captureObj->image_arr[buf]);
                if(status != VX_SUCCESS)
                {
                    TIOVX_MODULE_ERROR("[CAPTURE-MODULE] Unable to create RAW image object array! \n");
                    break;
                }
                else
                {
                    vx_char name[VX_MAX_REFERENCE_NAME];

                    snprintf(name, VX_MAX_REFERENCE_NAME, "capture_node_raw_image_arr_%d", buf);

                    vxSetReferenceName((vx_reference)captureObj->image_arr[buf], name);
                }
            }
            tivxReleaseRawImage(&raw_image);
        }
        else
        {
            TIOVX_MODULE_ERROR("[CAPTURE-MODULE] Unable to create RAW image object! \n");
        }
    }
    /* YUV format */
    else
    {
        vx_image cap_yuv_image = vxCreateImage(context, sensorParams->sensorInfo.raw_params.width, sensorParams->sensorInfo.raw_params.height, VX_DF_IMAGE_UYVY);
        status = vxGetStatus((vx_reference)cap_yuv_image);

        if(status == VX_SUCCESS)
        {
            for(buf = 0; buf < captureObj->out_bufq_depth; buf++)
            {
                captureObj->image_arr[buf] = vxCreateObjectArray(context, (vx_reference)cap_yuv_image, sensorObj->num_cameras_enabled);
                status = vxGetStatus((vx_reference)captureObj->image_arr[buf]);
                if(status != VX_SUCCESS)
                {
                    TIOVX_MODULE_ERROR("[CAPTURE-MODULE] Unable to create YUV image object array! \n");
                    break;
                }
                else
                {
                    vx_char name[VX_MAX_REFERENCE_NAME];

                    snprintf(name, VX_MAX_REFERENCE_NAME, "capture_node_yuv_image_arr_%d", buf);

                    vxSetReferenceName((vx_reference)captureObj->image_arr[buf], name);
                }
            }
            vxReleaseImage(&cap_yuv_image);
        }
        else
        {
            TIOVX_MODULE_ERROR("[CAPTURE-MODULE] Unable to create YUV image object! \n");
        }
    }

    if(captureObj->en_out_image_write == 1)
    {
        char file_path[TIVX_FILEIO_FILE_PATH_LENGTH];
        char file_prefix[TIVX_FILEIO_FILE_PREFIX_LENGTH];

        strcpy(file_path, captureObj->output_file_path);
        captureObj->file_path   = vxCreateArray(context, VX_TYPE_UINT8, TIVX_FILEIO_FILE_PATH_LENGTH);
        status = vxGetStatus((vx_reference)captureObj->file_path);
        if(status == VX_SUCCESS)
        {
            vxAddArrayItems(captureObj->file_path, TIVX_FILEIO_FILE_PATH_LENGTH, &file_path[0], 1);
        }
        else
        {
            TIOVX_MODULE_ERROR("[CAPTURE-MODULE] Unable to create file path object for fileio!\n");
        }

        sprintf(file_prefix, "capture_output");
        captureObj->file_prefix = vxCreateArray(context, VX_TYPE_UINT8, TIVX_FILEIO_FILE_PREFIX_LENGTH);
        status = vxGetStatus((vx_reference)captureObj->file_prefix);
        if(status == VX_SUCCESS)
        {
            vxAddArrayItems(captureObj->file_prefix, TIVX_FILEIO_FILE_PREFIX_LENGTH, &file_prefix[0], 1);
        }
        else
        {
            TIOVX_MODULE_ERROR("[CAPTURE-MODULE] Unable to create file prefix object for output!\n");
        }

        captureObj->write_cmd = vxCreateUserDataObject(context, "tivxFileIOWriteCmd", sizeof(tivxFileIOWriteCmd), NULL);
        status = vxGetStatus((vx_reference)captureObj->write_cmd);
        if(status == VX_SUCCESS)
        {
            TIOVX_MODULE_ERROR("[CAPTURE-MODULE] Unable to create write cmd object for output!\n");
        }
    }
    else
    {
        captureObj->file_path   = NULL;
        captureObj->file_prefix = NULL;
        captureObj->write_node  = NULL;
        captureObj->write_cmd   = NULL;
    }

    return status;
}

static vx_status tiovx_capture_module_create_config(vx_context context, TIOVXCaptureModuleObj *obj)
{
    vx_status status = VX_SUCCESS;

    obj->config = vxCreateUserDataObject(context, "tivx_capture_params_t", sizeof(tivx_capture_params_t), &obj->params);
    status = vxGetStatus((vx_reference)obj->config);

    if(VX_SUCCESS == status)
    {
        vxSetReferenceName((vx_reference)obj->config, "capture_node_config");
    }

    return status;
}

/*  This function takes a raw_image that is unpopulated
    and populates it with the path set below */
static tivx_raw_image read_error_image_raw(vx_context context,
                             IssSensor_Info *sensorInfo, char raw_image_fname[],
                             vx_int32 *bytes_read)
{
    FILE * fp;
    vx_uint32 width, height, i;
    vx_imagepatch_addressing_t image_addr;
    vx_rectangle_t rect;
    vx_map_id map_id;
    void *data_ptr;
    vx_uint32 num_bytes_per_pixel = 2; /*Supports only RAW 12b Unpacked format*/
    vx_uint32 num_bytes_read_from_file;
    tivx_raw_image raw_image = NULL;
    tivx_raw_image_format_t format;
    vx_uint32 imgaddr_width, imgaddr_height, imgaddr_stride;
    vx_status status = VX_SUCCESS;

    /* Nothing is being populated here - just an empty frame */
    raw_image = tivxCreateRawImage(context, &(sensorInfo->raw_params));

    status = vxGetStatus((vx_reference)raw_image);

    if(status == VX_SUCCESS)
    {
        vxSetReferenceName((vx_reference)raw_image, "capture_node_error_frame_raw_image");
    }
    else
    {
        TIOVX_MODULE_ERROR("[CAPTURE_MODULE] Unable to create error frame RAW image!\n");
    }

    if ((vx_status)VX_SUCCESS == status)
    {
        tivxQueryRawImage(raw_image, TIVX_RAW_IMAGE_WIDTH, &width, sizeof(vx_uint32));
        tivxQueryRawImage(raw_image, TIVX_RAW_IMAGE_HEIGHT, &height, sizeof(vx_uint32));
        tivxQueryRawImage(raw_image, TIVX_RAW_IMAGE_FORMAT, &format, sizeof(format));

        rect.start_x = 0;
        rect.start_y = 0;
        rect.end_x = width;
        rect.end_y = height;

        tivxMapRawImagePatch(raw_image,
            &rect,
            0,
            &map_id,
            &image_addr,
            &data_ptr,
            VX_WRITE_ONLY,
            VX_MEMORY_TYPE_HOST,
            TIVX_RAW_IMAGE_PIXEL_BUFFER
            );

        if(!data_ptr)
        {
            TIOVX_MODULE_ERROR("data_ptr is NULL \n");
            tivxReleaseRawImage(&raw_image);
            return NULL;
        }

        TIOVX_MODULE_PRINTF("Reading test RAW image %s \n", raw_image_fname);
        fp = fopen(raw_image_fname, "rb");

        if(!fp)
        {
            TIOVX_MODULE_ERROR("read_test_image_raw : Unable to open file %s, setting error message as all 0s\n", raw_image_fname);
            memset(data_ptr, 0x00, image_addr.stride_y*height);
            *bytes_read = 0;
        }
        else
        {
            num_bytes_read_from_file = 0;

            imgaddr_width  = image_addr.dim_x;
            imgaddr_height = image_addr.dim_y;
            imgaddr_stride = image_addr.stride_y;

            for(i=0;i<imgaddr_height;i++)
            {
                num_bytes_read_from_file += fread(data_ptr, 1, imgaddr_width*num_bytes_per_pixel, fp);
                data_ptr += imgaddr_stride;
            }

            fclose(fp);

            TIOVX_MODULE_PRINTF("%d bytes read from %s\n", num_bytes_read_from_file, raw_image_fname);
            *bytes_read = num_bytes_read_from_file;
        }

        tivxUnmapRawImagePatch(raw_image, map_id);
    }
    return raw_image;
}

static vx_status tiovx_capture_module_create_error_detection_frame(vx_context context, TIOVXCaptureModuleObj *captureObj, SensorObj *sensorObj)
{
    vx_status status = VX_SUCCESS;
    IssSensor_CreateParams *sensorParams = &sensorObj->sensorParams;

    /*Error detection is currently enabled only for RAW input*/
    if(0 != captureObj->capture_format)
    {
        captureObj->enable_error_detection = 0;
    }

    /* If error detection is enabled, send the test frame */
    if (1 == captureObj->enable_error_detection)
    {
        uint32_t path_index = 0;
        vx_int32 bytes_read = 0;
#if defined (QNX)
        const char * test_data_path = "/ti_fs/vision_apps/test_data/";
#elif defined (PC)
        const char * test_data_path = "./";
#else
        const char * test_data_path = "/opt/vision_apps/test_data/";
#endif
        char raw_image_fname[256] = {0};
        const char test_image_paths[3][64] =
        {
            "psdkra/app_single_cam/IMX390_001/input2",  /* Used in test mode */
            "psdkra/app_single_cam/AR0233_001/input2",  /* Used in test mode */
            "img_test"                                  /* Used as error frame when not in test mode */
        };

        /* Point to last index in the test_image_paths list */
        path_index = ((sizeof(test_image_paths)/64)/sizeof(char))-1;

        snprintf(raw_image_fname, 256, "%s/%s.raw", test_data_path, test_image_paths[path_index]);

        captureObj->error_frame_raw_image = read_error_image_raw(context, &(sensorParams->sensorInfo),
                                                raw_image_fname,
                                                &bytes_read);

        TIOVX_MODULE_PRINTF("%d bytes were read by read_error_image_raw() from path %s\n", bytes_read, test_data_paths[sensorObj->sensor_index]);
        status = vxGetStatus((vx_reference)captureObj->error_frame_raw_image);

        if(status == VX_SUCCESS)
        {
            if(bytes_read < 0)
            {
                TIOVX_MODULE_PRINTF("[CAPTURE_MODULE] Bytes read by error frame for RAW image is < 0! \n");
                tivxReleaseRawImage(&captureObj->error_frame_raw_image);
                captureObj->error_frame_raw_image = NULL; /* Is this required after releasing the reference? */
            }
        }
        else
        {
            TIOVX_MODULE_PRINTF("[CAPTURE_MODULE] Unable to create error frame RAW image!\n");
        }
    }
    else
    {
        captureObj->error_frame_raw_image = NULL;
    }

    return status;
}

void tiovx_capture_module_params_init(TIOVXCaptureModuleObj *captureObj, SensorObj *sensorObj)
{
    vx_status status = VX_SUCCESS;

    vx_uint32   num_capt_instances = 0;
    vx_int32    id, lane, ch, vcNum;
    int32_t     ch_mask = sensorObj->ch_mask;

    if (ch_mask <= 0xF)
    {
        num_capt_instances = 1;
    }
    else if ((ch_mask > 0xF) && (ch_mask <= 0xFF))
    {
        num_capt_instances = 2;
    }
    #if defined(SOC_J784S4)
    else if ((ch_mask > 0xFF) && (ch_mask <= 0xFFF))
    {
        num_capt_instances = 3;
    }
    #endif
    else
    {
        TIOVX_MODULE_ERROR("[CAPTURE_MODULE] - ch_mask parameter is invalid! \n");
        status = VX_ERROR_INVALID_PARAMETERS;
    }

    if(status == VX_SUCCESS)
    {
        captureObj->capture_format = sensorObj->sensor_out_format;

        tivx_capture_params_init(&captureObj->params);

        if (captureObj->enable_error_detection)
        {
            captureObj->params.timeout        = 90;
            captureObj->params.timeoutInitial = 500;
        }
        captureObj->params.numInst  = num_capt_instances;
        captureObj->params.numCh    = sensorObj->num_cameras_enabled;

        for(id = 0; id < num_capt_instances; id++)
        {
            captureObj->params.instId[id]                       = id;
            captureObj->params.instCfg[id].enableCsiv2p0Support = (uint32_t)vx_true_e;
            captureObj->params.instCfg[id].numDataLanes         = sensorObj->sensorParams.sensorInfo.numDataLanes;
            captureObj->params.instCfg[id].laneBandSpeed        = sensorObj->sensorParams.sensorInfo.csi_laneBandSpeed;

            TIOVX_MODULE_PRINTF("captureObj->params.numDataLanes = %d \n", 
                                    captureObj->params.instCfg[id].numDataLanes);

            for (lane = 0; lane < captureObj->params.instCfg[id].numDataLanes; lane++)
            {
                captureObj->params.instCfg[id].dataLanesMap[lane] = lane + 1;
                TIOVX_MODULE_PRINTF("captureObj->params.dataLanesMap[%d] = %d \n", lane,
                                    captureObj->params.instCfg[id].dataLanesMap[lane]);
            }
        }

        ch = 0;     /*Camera Physical Channel Number*/
        vcNum = 0;  /*CSI2 Virtual Channel Number*/
        id = 0;     /*CSI2 Instance ID*/
        
        while(ch_mask > 0)
        {
            if(ch > 7)
            {
                id = 2;
            }
            else if( (ch > 3) && (ch <= 7) )
            {
                id = 1;
            }
            else
            {
                id = 0;
            }

            if(ch_mask & 0x1)
            {
                captureObj->params.chVcNum[vcNum] = ch%4;
                captureObj->params.chInstMap[vcNum] = id;
                vcNum++;
            }

            ch++;
            ch_mask = ch_mask >> 1;
        }
    }
}


vx_status tiovx_capture_module_init(vx_context context, TIOVXCaptureModuleObj *captureObj, SensorObj *sensorObj)
{
    vx_status status = VX_SUCCESS;

    if((vx_status)VX_SUCCESS == status)
    {
        status = tiovx_capture_module_create_config(context, captureObj);
    }

    if((vx_status)VX_SUCCESS == status)
    {
        status = tiovx_capture_module_create_output(context, captureObj, sensorObj);
    }

    if(status == VX_SUCCESS)
    {
        status = tiovx_capture_module_create_error_detection_frame(context, captureObj, sensorObj);
    }

    return status;
}

vx_status tiovx_capture_module_deinit(TIOVXCaptureModuleObj *obj)
{
    vx_status status = VX_SUCCESS;
    vx_int32 buf;

    TIOVX_MODULE_PRINTF("[CAPTURE-MODULE] Releasing config handle!\n");
    status = vxReleaseUserDataObject(&obj->config);

    for(buf = 0; buf < obj->out_bufq_depth; buf++)
    {
        if((vx_status)VX_SUCCESS == status)
        {
            TIOVX_MODULE_PRINTF("[CAPTURE-MODULE] Releasing output array of bufq %d!\n", buf);
            status = vxReleaseObjectArray(&obj->image_arr[buf]);
        }
    }

    if(obj->en_out_image_write == 1)
    {
        if((vx_status)VX_SUCCESS == status)
        {
            TIOVX_MODULE_PRINTF("[CAPTURE-MODULE] Releasing output path array!\n");
            status = vxReleaseArray(&obj->file_path);
        }

        if((vx_status)VX_SUCCESS == status)
        {
            TIOVX_MODULE_PRINTF("[CAPTURE-MODULE] Releasing output file prefix array!\n");
            status = vxReleaseArray(&obj->file_prefix);
        }
        if((vx_status)VX_SUCCESS == status)
        {
            TIOVX_MODULE_PRINTF("[CAPTURE-MODULE] Releasing output write command object!\n");
            status = vxReleaseUserDataObject(&obj->write_cmd);
        }
    }

    return status;
}

vx_status tiovx_capture_module_delete(TIOVXCaptureModuleObj *obj)
{
    vx_status status = VX_SUCCESS;

    if(obj->node != NULL)
    {
        TIOVX_MODULE_PRINTF("[CAPTURE-MODULE] Releasing node reference!\n");
        status = vxReleaseNode(&obj->node);
    }

    return status;
}

vx_status tiovx_capture_module_create(vx_graph graph, TIOVXCaptureModuleObj *obj, const char* target_string)
{
    vx_status status = VX_SUCCESS;

    obj->node = tivxCaptureNode(graph, obj->config, obj->image_arr[0]);
    status = vxGetStatus((vx_reference)obj->node);

    if(status == VX_SUCCESS)
    {
        vxSetReferenceName((vx_reference)obj->node, "capture_node");
        vxSetNodeTarget(obj->node, VX_TARGET_STRING, target_string);

        if(obj->en_out_image_write == 1)
        {
            if(obj->image_arr[0] != NULL)
            {
                status = tiovx_capture_module_add_write_output_node(graph, obj);
                if(status != VX_SUCCESS)
                {
                    TIOVX_MODULE_ERROR("[CAPTURE-MODULE] Unable to create write node for output!\n");
                }
            }
        }
    }
    else
    {
        TIOVX_MODULE_ERROR("[CAPTURE-MODULE] Unable to create capture node! \n");
    }

    return status;
}

vx_status tiovx_capture_module_release_buffers(TIOVXCaptureModuleObj *obj)
{
    vx_status status = VX_SUCCESS;

    void        *virtAddr[TIOVX_MODULES_MAX_REF_HANDLES] = {NULL};
    vx_uint32   size[TIOVX_MODULES_MAX_REF_HANDLES];
    vx_uint32   numEntries;
    vx_int32    bufq;

    for(bufq = 0; bufq < obj->out_bufq_depth; bufq++)
    {
        vx_reference ref = (vx_reference)obj->image_arr;
        status = vxGetStatus((vx_reference)ref);

        if((vx_status)VX_SUCCESS == status)
        {
            /* Export handles to get valid size information. */
            status = tivxReferenceExportHandle(ref,
                                                virtAddr,
                                                size,
                                                TIOVX_MODULES_MAX_REF_HANDLES,
                                                &numEntries);

            if((vx_status)VX_SUCCESS == status)
            {
                vx_int32 ctr;
                /* Currently the vx_image buffers are alloated in one shot for multiple planes.
                    So if we are freeing this buffer then we need to get only the first plane
                    pointer address but add up the all the sizes to free the entire buffer */
                vx_uint32 freeSize = 0;
                for(ctr = 0; ctr < numEntries; ctr++)
                {
                    freeSize += size[ctr];
                }

                if(virtAddr[0] != NULL)
                {
                    TIOVX_MODULE_PRINTF("[CAPTURE-MODULE] Freeing output, bufq=%d, addr = 0x%016lX, size = %d \n", bufq, (vx_uint64)virtAddr[0], freeSize);
                    tivxMemFree(virtAddr[0], freeSize, TIVX_MEM_EXTERNAL);
                }

                for(ctr = 0; ctr < numEntries; ctr++)
                {
                    virtAddr[ctr] = NULL;
                }

                /* Assign NULL handles to the OpenVx objects as it will avoid
                    doing a tivxMemFree twice, once now and once during release */
                status = tivxReferenceImportHandle(ref,
                                                (const void **)virtAddr,
                                                (const uint32_t *)size,
                                                numEntries);
            }
        }
    }

    if ((vx_status)VX_SUCCESS != status)
    {
        TIOVX_MODULE_ERROR("tivxReferenceExportHandle() failed.\n");
    }

    return status;
}

vx_status tiovx_capture_module_add_write_output_node(vx_graph graph, TIOVXCaptureModuleObj *obj)
{
    vx_status status = VX_SUCCESS;

    /* RAW */
    if(0 == obj->capture_format)
    {
        tivx_raw_image raw_img = (tivx_raw_image)vxGetObjectArrayItem(obj->image_arr[0], 0);
        obj->write_node = tivxWriteRawImageNode(graph, raw_img, obj->file_path, obj->file_prefix);
        tivxReleaseRawImage(&raw_img);

        status = vxGetStatus((vx_reference)obj->write_node);
        if(status == VX_SUCCESS)
        {
            vxSetReferenceName((vx_reference)obj->write_node, "capture_write_node");
            vxSetNodeTarget(obj->write_node, VX_TARGET_STRING, TIVX_TARGET_MPU_0);

            vx_bool replicate[] = { vx_true_e, vx_false_e, vx_false_e};
            vxReplicateNode(graph, obj->write_node, replicate, 3);
        }
        else
        {
            TIOVX_MODULE_ERROR("[CAPTURE-MODULE] Unable to create RAW image write node! \n");
        }
    }
    /* YUV */
    else
    {
        vx_image yuv_img = (vx_image)vxGetObjectArrayItem(obj->image_arr[0], 0);
        obj->write_node = tivxWriteImageNode(graph, yuv_img, obj->file_path, obj->file_prefix);
        vxReleaseImage(&yuv_img);

        status = vxGetStatus((vx_reference)obj->write_node);
        if(status == VX_SUCCESS)
        {
            vxSetReferenceName((vx_reference)obj->write_node, "capture_write_node");
            vxSetNodeTarget(obj->write_node, VX_TARGET_STRING, TIVX_TARGET_MPU_0);

            vx_bool replicate[] = { vx_true_e, vx_false_e, vx_false_e};
            vxReplicateNode(graph, obj->write_node, replicate, 3);
        }
        else
        {
            TIOVX_MODULE_ERROR("[CAPTURE-MODULE] Unable to create YUV image write node! \n");
        }
    }
    return (status);
}

vx_status tiovx_capture_module_send_write_output_cmd(TIOVXCaptureModuleObj *obj, vx_uint32 start_frame, vx_uint32 num_frames, vx_uint32 num_skip)
{
    vx_status status = VX_SUCCESS;

    tivxFileIOWriteCmd write_cmd;

    write_cmd.start_frame = start_frame;
    write_cmd.num_frames = num_frames;
    write_cmd.num_skip = num_skip;

     status = vxCopyUserDataObject(obj->write_cmd, 0, sizeof(tivxFileIOWriteCmd),\
                  &write_cmd, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);

    if(status == VX_SUCCESS)
    {
        vx_reference refs[2];

        refs[0] = (vx_reference)obj->write_cmd;

        status = tivxNodeSendCommand(obj->write_node, TIVX_CONTROL_CMD_SEND_TO_ALL_REPLICATED_NODES,
                                 TIVX_FILEIO_CMD_SET_FILE_WRITE,
                                 refs, 1u);

        if(VX_SUCCESS != status)
        {
            TIOVX_MODULE_ERROR("Capture node send command failed!\n");
        }

        TIOVX_MODULE_PRINTF("Capture node send command success!\n");
    }


    return (status);
}