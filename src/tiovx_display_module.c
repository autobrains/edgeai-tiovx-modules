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

#include <tiovx_display_module.h>

static vx_status tiovx_display_module_create_input(vx_context context, TIOVXDisplayModuleObj *obj)
{
    vx_status status = VX_SUCCESS;

    obj->input = vxCreateImage(context, obj->input_width, obj->input_height, obj->input_color_format);
    status = vxGetStatus((vx_reference)obj->input);

    if(status != VX_SUCCESS)
    {
        TIOVX_MODULE_ERROR("[DISPLAY-MODULE] Unable to create input image! \n");
    }
    else
    {
        vx_char name[VX_MAX_REFERENCE_NAME];

        snprintf(name, VX_MAX_REFERENCE_NAME, "display_node_input_image");
        vxSetReferenceName((vx_reference)obj->input, name);
    }

    return status;
}

static vx_status tiovx_display_module_create_config(vx_context context, TIOVXDisplayModuleObj *obj)
{
    vx_status status = VX_SUCCESS;

    obj->config = vxCreateUserDataObject(context, "tivx_display_params_t", sizeof(tivx_display_params_t), &obj->params);
    status = vxGetStatus((vx_reference)obj->config);

    if(VX_SUCCESS == status)
    {
        vxSetReferenceName((vx_reference)obj->config, "display_node_config");
    }

    return status;
}

void tiovx_display_module_params_init(TIOVXDisplayModuleObj *obj)
{
    memset(&obj->params, 0, sizeof(tivx_display_params_t));

    obj->params.opMode = TIVX_KERNEL_DISPLAY_ZERO_BUFFER_COPY_MODE; //TIVX_KERNEL_DISPLAY_BUFFER_COPY_MODE;
    obj->params.pipeId = 0; /* pipe ID = 2 */
    obj->params.outWidth = DEFAULT_DISPLAY_WIDTH;
    obj->params.outHeight = DEFAULT_DISPLAY_HEIGHT;
    obj->params.posX = 0;
    obj->params.posY = 0;
}

vx_status tiovx_display_module_init(vx_context context, TIOVXDisplayModuleObj *obj)
{
    vx_status status = VX_SUCCESS;

    if((vx_status)VX_SUCCESS == status)
    {
        status = tiovx_display_module_create_config(context, obj);
    }

    if((vx_status)VX_SUCCESS == status)
    {
        status = tiovx_display_module_create_input(context, obj);
    }

    return status;
}

vx_status tiovx_display_module_deinit(TIOVXDisplayModuleObj *obj)
{
    vx_status status = VX_SUCCESS;

    TIOVX_MODULE_PRINTF("[DISPLAY-MODULE] Releasing config handle!\n");
    status = vxReleaseUserDataObject(&obj->config);
    
    if(status == VX_SUCCESS)
    {
        status = vxReleaseImage(&obj->input);
    }

    return status;
}

vx_status tiovx_display_module_delete(TIOVXDisplayModuleObj *obj)
{
    vx_status status = VX_SUCCESS;

    if(obj->node != NULL)
    {
        TIOVX_MODULE_PRINTF("[DISPLAY-MODULE] Releasing node reference!\n");
        status = vxReleaseNode(&obj->node);
    }

    return status;
}

vx_status tiovx_display_module_create(vx_graph graph, TIOVXDisplayModuleObj *obj, vx_image input_image, const char* target_string)
{
    vx_status status = VX_SUCCESS;

    vx_image input;

    if(input_image != NULL)
    {
        input = input_image;
    }
    else
    {
        if(obj->input != NULL)
        {
            input = obj->input;
        }
        else
        {
            input = NULL;
        }
    }

    obj->node = tivxDisplayNode(graph, obj->config, input);
    status = vxGetStatus((vx_reference)obj->node);

    if(status == VX_SUCCESS)
    {
        vxSetReferenceName((vx_reference)obj->node, "DisplayNode");
        vxSetNodeTarget(obj->node, VX_TARGET_STRING, target_string);
    }
    else
    {
        TIOVX_MODULE_ERROR("[DISPLAY-MODULE] Unable to create display node!\n");
    }

    return status;
}

vx_status tiovx_display_module_release_buffers(TIOVXDisplayModuleObj *obj)
{
    vx_status status = VX_SUCCESS;

    void        *virtAddr[TIOVX_MODULES_MAX_REF_HANDLES] = {NULL};
    vx_uint32   size[TIOVX_MODULES_MAX_REF_HANDLES];
    vx_uint32   numEntries;

    /* Free input handle */
    vx_reference ref = (vx_reference)obj->input;
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
                TIOVX_MODULE_PRINTF("[DISPLAY-MODULE] Freeing input, addr = 0x%016lX, size = %d \n", (vx_uint64)virtAddr[0], freeSize);
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

    if ((vx_status)VX_SUCCESS != status)
    {
        VX_PRINT(VX_ZONE_ERROR, "tivxReferenceExportHandle() failed.\n");
    }

    return status;
}
