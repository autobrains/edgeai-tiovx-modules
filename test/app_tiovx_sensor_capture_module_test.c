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

#include <tiovx_utils.h>
#include <tiovx_sensor_module.h>
#include <tiovx_capture_module.h>
#include <TI/video_io_kernels.h>

#define APP_BUFQ_DEPTH   (1)
#define NUM_ITERATIONS   (4)

#define APP_NUM_CH       (1)
#define APP_NUM_OUTPUTS  (1)

typedef struct {

    /* OpenVX references */
    vx_context context;

    vx_graph   graph;

    vx_int32   output_obj_array_graph_parameter_index;

    SensorObj  sensorObj;

    TIOVXCaptureModuleObj   captureObj;

} AppObj;

static AppObj gAppObj;

static vx_status app_init(AppObj *obj);
static void app_deinit(AppObj *obj);
static vx_status app_create_graph(AppObj *obj);
static vx_status app_verify_graph(AppObj *obj);
static vx_status app_run_graph(AppObj *obj);
static void app_delete_graph(AppObj *obj);

vx_status app_modules_sensor_capture_test(vx_int32 argc, vx_char* argv[])
{
    AppObj *obj = &gAppObj;
    vx_status status = VX_SUCCESS;

    status = app_init(obj);
    APP_PRINTF("App Init Done! \n");

    if(status == VX_SUCCESS)
    {
        status = app_create_graph(obj);
        APP_PRINTF("App Create Graph Done! \n");
    }

    if(status == VX_SUCCESS)
    {
        status = app_verify_graph(obj);
        APP_PRINTF("App Verify Graph Done! \n");
    }
    if (status == VX_SUCCESS)
    {
        status = app_run_graph(obj);
        APP_PRINTF("App Run Graph Done! \n");
    }

    app_delete_graph(obj);
    APP_PRINTF("App Delete Graph Done! \n");

    app_deinit(obj);
    APP_PRINTF("App De-init Done! \n");

    return status;
}

static vx_status app_init(AppObj *obj)
{
    vx_status status = VX_SUCCESS;

    /* Create OpenVx Context */
    obj->context = vxCreateContext();
    status = vxGetStatus((vx_reference) obj->context);

    if(status == VX_SUCCESS)
    {
        tivxVideoIOLoadKernels(obj->context);
    }

    /* Sensor Init */
    if(status == VX_SUCCESS)
    {
        tiovx_sensor_module_params_init(&obj->sensorObj);

        /* For running the application in interactive mode
            This gives configurability to users for selecting 
            the sensor at runtime.
            obj->sensorObj.is_interactive = 1; */
        obj->sensorObj.is_interactive = 0;

        /* ch_mask = 00001101 means cameras connected to port 
            (or m_cameraId) 0, 2 and 3 of fusion board */
        obj->sensorObj.ch_mask = 1;

        tiovx_sensor_module_query(&obj->sensorObj);

        /* sensor_index is zero for IMX390 2MP cameras */
        obj->sensorObj.sensor_index = 0;

        /* Initialize sensor module */
        status = tiovx_sensor_module_init(&obj->sensorObj, "sensor_obj");
    }

    /* Capture Init */
    if(status == VX_SUCCESS)
    {
        tiovx_capture_module_params_init(&obj->captureObj, &obj->sensorObj);
        obj->captureObj.enable_error_detection = 0; /* or 1 */
        obj->captureObj.out_bufq_depth = 4; /* This must be greater than 3 */

        /* Initialize capture module */
        status = tiovx_capture_module_init(obj->context, &obj->captureObj,
                                            &obj->sensorObj);
        APP_PRINTF("Capture Init Done! \n");
    }

    return status;
}

static void app_deinit(AppObj *obj)
{
    tiovx_sensor_module_deinit(&obj->sensorObj);
    tiovx_capture_module_deinit(&obj->captureObj);

    tivxVideoIOUnLoadKernels(obj->context);

    vxReleaseContext(&obj->context);
}

static void app_delete_graph(AppObj *obj)
{
    tiovx_capture_module_delete(&obj->captureObj);

    vxReleaseGraph(&obj->graph);
}

static vx_status app_create_graph(AppObj *obj)
{
    vx_status status = VX_SUCCESS;

    vx_graph_parameter_queue_params_t graph_parameters_queue_params_list[8];
    vx_int32 graph_parameter_index;

    obj->graph = vxCreateGraph(obj->context);
    status = vxGetStatus((vx_reference)obj->graph);

    if((vx_status)VX_SUCCESS == status)
    {
        status = tiovx_capture_module_create(obj->graph,
                                            &obj->captureObj,
                                            TIVX_TARGET_CAPTURE1);
    }

    graph_parameter_index = 0;

    if((vx_status)VX_SUCCESS == status)
    {
        status = add_graph_parameter_by_node_index(obj->graph, obj->captureObj.node, 1);

        obj->captureObj.graph_parameter_index = graph_parameter_index;
        graph_parameters_queue_params_list[graph_parameter_index].graph_parameter_index = graph_parameter_index;
        graph_parameters_queue_params_list[graph_parameter_index].refs_list_size = obj->captureObj.out_bufq_depth;
        graph_parameters_queue_params_list[graph_parameter_index].refs_list = (vx_reference*)&obj->captureObj.image_arr[0];
        graph_parameter_index++;
    }

    if((vx_status)VX_SUCCESS == status)
    {
        status = vxSetGraphScheduleConfig(obj->graph,
                    VX_GRAPH_SCHEDULE_MODE_QUEUE_AUTO,
                    graph_parameter_index,
                    graph_parameters_queue_params_list);
    }

    return status;
}

static vx_status app_verify_graph(AppObj *obj)
{
    vx_status status = VX_SUCCESS;

    status = vxVerifyGraph(obj->graph);

    APP_PRINTF("App Verify Graph Done!\n");

    return status;
}

static vx_status app_run_graph(AppObj *obj)
{
    vx_status status = VX_SUCCESS;

    char output_filename[100];

    sprintf(output_filename, "%s/output/imx390_1936x1096_capture_nv12.yuv", EDGEAI_DATA_PATH);

    vx_object_array output_obj_arr;
    tivx_raw_image output_o;

    uint32_t num_refs;
    int32_t frame_count;

    status = tiovx_sensor_module_start(&obj->sensorObj);

    for(frame_count=0; frame_count < obj->captureObj.out_bufq_depth; frame_count++)
    {
        if(status == VX_SUCCESS)
        {
            status = vxGraphParameterEnqueueReadyRef(obj->graph, obj->captureObj.graph_parameter_index,
                                            (vx_reference*)&obj->captureObj.image_arr[frame_count], 1);
        }
    }

    frame_count = 0;
    while (frame_count < NUM_ITERATIONS)
    {

        vxGraphParameterDequeueDoneRef(obj->graph, obj->captureObj.graph_parameter_index, (vx_reference*)&output_obj_arr, 1, &num_refs);

        output_o = (tivx_raw_image)vxGetObjectArrayItem((vx_object_array)obj->captureObj.image_arr[frame_count], 0);
        writeRawImage(output_filename, output_o);
        tivxReleaseRawImage(&output_o);

        vxGraphParameterEnqueueReadyRef(obj->graph, obj->captureObj.graph_parameter_index, (vx_reference*)&output_obj_arr, 1);

        frame_count++;
    }

    status = tiovx_sensor_module_stop(&obj->sensorObj);

    return status;
}
