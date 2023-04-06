/*
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All rights reserved.
 *
 * PROPRIETARY/CONFIDENTIAL.  USE IS SUBJECT TO LICENSE TERMS.
 */

#include <stdio.h>

#include "gadget_test_app.h"
#include "trace.h"
#include "string.h"
#include "gpio_handle.h"
/*
    Section: Gadget Include
*/
#include "pb_encode.h"
#include "pb_decode.h"
#include "directiveParser.pb.h"
#include "eventParser.pb.h"
#include "alexaDiscoveryDiscoverResponseEvent.pb.h"
#include "alexaDiscoveryDiscoverDirective.pb.h"
#include "alexaGadgetMusicDataTempoDirective.pb.h"
#include "alertsDeleteAlertDirective.pb.h"
#include "alertsSetAlertDirective.pb.h"
#include "alexaGadgetSpeechDataSpeechmarksDirective.pb.h"
#include "alexaGadgetStateListenerStateUpdateDirective.pb.h"
#include "notificationsSetIndicatorDirective.pb.h"
#include "notificationsClearIndicatorDirective.pb.h"
#include "crypto.h"

/*
    Section: Gadget Globals
*/

directive_DirectiveParserProto directive_parser_envelope =
    directive_DirectiveParserProto_init_default;
alexaDiscovery_DiscoverDirectiveProto discover_directive_envelope =
    alexaDiscovery_DiscoverDirectiveProto_init_default;
alexaDiscovery_DiscoverResponseEventProto discover_response_event_envelope =
    alexaDiscovery_DiscoverResponseEventProto_init_default;
alerts_DeleteAlertDirectiveProto deletealert_envelope =
    alerts_DeleteAlertDirectiveProto_init_default;
alerts_SetAlertDirectiveProto setalert_envelope = alerts_SetAlertDirectiveProto_init_default;
alexaGadgetMusicData_TempoDirectiveProto tempo_envelope =
    alexaGadgetMusicData_TempoDirectiveProto_init_default;
alexaGadgetSpeechData_SpeechmarksDirectiveProto speechmarks_envelope =
    alexaGadgetSpeechData_SpeechmarksDirectiveProto_init_default;
alexaGadgetStateListener_StateUpdateDirectiveProto stateupdate_envelope =
    alexaGadgetStateListener_StateUpdateDirectiveProto_init_default;
notifications_SetIndicatorDirectiveProto setnotifications_envelope =
    notifications_SetIndicatorDirectiveProto_init_default;
notifications_ClearIndicatorDirectiveProto clearnotifications_envelope =
    notifications_ClearIndicatorDirectiveProto_init_default;

uint8_t buffer[512];
uint8_t gadgetDSN[32] = "RTKGadget";
char *gadgetAmazonDeviceType = "A1TW6OJDO08AL0";
char *gadgetAmazonDeviceToken = "2EB4399E74FACCE478E5C274C75E22B0B3F6DA4BA76E08530408F865EBFD9430";
uint8_t hashOutBuffer[32];
uint32_t ulHashLength = sizeof(hashOutBuffer);
char hexArray[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
bool isTimerExist = false;

/*
    Section: Business Logic
*/

void gadget_set_name(char *deviceName)
{
    // Use device name as DSN for demo purpose.
    size_t max_len = sizeof(gadgetDSN);
    strncpy(gadgetDSN, deviceName, max_len);
    gadgetDSN[max_len - 1] = '\0';
}

static void createDeviceSecret(char *deviceDSN, char *amazonDeviceToken, char *deviceSecret,
                               uint8_t deviceSecretLength)
{
    uint32_t index = 0;
    uint8_t xErrCode = 0;
    uint8_t *hashInBuffer = NULL;
//    char c;

    hashInBuffer = (uint8_t *) malloc(sizeof(uint8_t) * (strlen(deviceDSN) + strlen(
                                                             amazonDeviceToken)));
    for (index = 0; index < strlen(deviceDSN); index++)
    {
        hashInBuffer[index] = *(deviceDSN + index);
    }
    for (index = 0; index < strlen(amazonDeviceToken); index++)
    {
        hashInBuffer[strlen(deviceDSN) + index] = *(amazonDeviceToken + index);
    }

    xComputeSHA256Hash(hashInBuffer, strlen(deviceDSN) + strlen(amazonDeviceToken), &hashOutBuffer,
                       &ulHashLength);
    if (xErrCode == 0)
    {
        DBG_DIRECT("\nHASH SUCCESS\n");
        int v;
        for (index = 0; index < 32; index++)
        {
            v = hashOutBuffer[index] & 0xFF;
            *(deviceSecret + (index * 2)) = hexArray[v >> 4];
            *(deviceSecret + (index * 2 + 1)) = hexArray[v & 0x0F];
        }
        *(deviceSecret + (index * 2)) = '\0';
    }
    else
    {
        DBG_DIRECT("\nHASH ERROR\n");
    }

    free(hashInBuffer);
    hashInBuffer = NULL;
    return;
}

uint16_t encode_sample_discover_response_event(void)
{
    /*
    {
        "event": {
            "header": {
                "namespace": "Alexa.Discovery",
                "name": "Discover.Response",
                "messageId": ""
            },
            "payload": {
                "endpoints": [
                {
                    "endpointId": "nrf52840",
                    "friendlyName": "NordicBLE",
                    "description": "NordicDK",
                    "capabilities": [
                    {
                        "type": "AlexaInterface",
                        "interface": "lexa.Gadget.StateListener",
                        "version": "1.0",
                        "configuration": {
                            "supportedTypes": [
                            {
                                "name": "timeinfo"
                            },
                            {
                                "name": "timers"
                            },
                            {
                                "name": "alarms"
                            },
                            {
                                "name": "reminders"
                            },
                            {
                                "name": "wakeword"
                            }
                            ]
                        },
                    },
                    {
                        "type": "AlexaInterface",
                        "interface": "Alerts",
                        "version": "1.1",
                        "configuration": {
                            "supportedTypes": [
                           ]
                        }
                    },
                    {
                        "type": "AlexaInterface",
                        "interface": "Notifications",
                        "version": "1.0",
                        "configuration": {
                            "supportedTypes": [
                            ]
                        }
                    },
                    {
                        "type": "AlexaInterface",
                        "interface": "Alexa.Gadget.MusicData",
                        "version": "1.0",
                        "configuration": {
                            "supportedTypes": [
                            {
                                "name": "tempo"
                            }
                            ]
                        }
                    },
                    {
                        "type": "AlexaInterface",
                        "interface": "Alexa.Gadget.SpeechData",
                        "version": "1.0",
                        "configuration": {
                            "supportedTypes": [
                            {
                                  "name": "viseme"
                            }
                            ]
                        }
                    },
                    ],
                    "additionalIdentification": {
                          "firmwareVersion": "1",
                          "deviceToken": "xxxxxxxxx",
                          "deviceTokenEncryptionType": "1",
                          "amazonDeviceType": "aabbccd",
                          "modelName": "NordicBLEGadget",
                          "radioAddress": "default"
                    }
                }]
            }
        }
    }
    */
//    uint16_t index = 0;
    uint8_t capabilities_count;
    uint8_t supportedtypes_count;
//    uint8_t xErrCode = 0;
    uint8_t deviceSecretLength = 32;
    //Add an extra byte at the end to put a null terminator
    uint8_t *deviceSecret = (uint8_t *) calloc(sizeof(uint8_t), deviceSecretLength * 2 + 1);

    DBG_DIRECT("\nCreating discover response event:\n");
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    strcpy(discover_response_event_envelope.event.header.namespace, "Alexa.Discovery");
    strcpy(discover_response_event_envelope.event.header.name, "Discover.Response");

    discover_response_event_envelope.event.payload.endpoints_count = 1;
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].endpointId, gadgetDSN); //dsn
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].friendlyName,
           "RealtekBLE"); //gadgetname
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].description,
           "Bee2DK"); //not used in discovery
    //strcpy(discover_response_event_envelope.event.payload.endpoints[0].manufacturerName, "TestManu"); //manufacturer
    capabilities_count = 0;
    supportedtypes_count = 0;
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].type,
           "AlexaInterface");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].interface,
           "Alexa.Gadget.StateListener");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].version,
           "1.0");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].configuration.supportedTypes[supportedtypes_count++].name,
           "timeinfo");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].configuration.supportedTypes[supportedtypes_count++].name,
           "timers");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].configuration.supportedTypes[supportedtypes_count++].name,
           "alarms");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].configuration.supportedTypes[supportedtypes_count++].name,
           "reminders");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].configuration.supportedTypes[supportedtypes_count++].name,
           "wakeword");
    discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].configuration.supportedTypes_count
        = supportedtypes_count;
    capabilities_count++;
    supportedtypes_count = 0;
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].type,
           "AlexaInterface");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].interface,
           "Alerts");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].version,
           "1.1");
    discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].configuration.supportedTypes_count
        = supportedtypes_count;
    capabilities_count++;
    supportedtypes_count = 0;
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].type,
           "AlexaInterface");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].interface,
           "Notifications");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].version,
           "1.0");
    discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].configuration.supportedTypes_count
        = supportedtypes_count;
    capabilities_count++;
    supportedtypes_count = 0;
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].type,
           "AlexaInterface");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].interface,
           "Alexa.Gadget.MusicData");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].version,
           "1.0");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].configuration.supportedTypes[supportedtypes_count++].name,
           "tempo");
    discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].configuration.supportedTypes_count
        = supportedtypes_count;
    capabilities_count++;
    supportedtypes_count = 0;
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].type,
           "AlexaInterface");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].interface,
           "Alexa.Gadget.SpeechData");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].version,
           "1.0");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].configuration.supportedTypes[supportedtypes_count++].name,
           "viseme");
    discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].configuration.supportedTypes_count
        = supportedtypes_count;
    capabilities_count++;
    supportedtypes_count = 0;

    /*strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].type, "AlexaInterface");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].interface, "Custom.Event");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].version, "1.0");
    discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].configuration.supportedTypes_count = supportedtypes_count;
    capabilities_count++;
    supportedtypes_count = 0;
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].type, "AlexaInterface");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].interface, "Custom.Directive");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].version, "1.0");
    discover_response_event_envelope.event.payload.endpoints[0].capabilities[capabilities_count].configuration.supportedTypes_count = supportedtypes_count;
    capabilities_count++;
    supportedtypes_count = 0;*/

    discover_response_event_envelope.event.payload.endpoints[0].capabilities_count = capabilities_count;
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].additionalIdentification.firmwareVersion,
           "1");
    //Compute hash
    createDeviceSecret(gadgetDSN, gadgetAmazonDeviceToken, deviceSecret, deviceSecretLength);
    strncpy(discover_response_event_envelope.event.payload.endpoints[0].additionalIdentification.deviceToken,
            deviceSecret, deviceSecretLength * 2 + 1);
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].additionalIdentification.deviceTokenEncryptionType,
           "1");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].additionalIdentification.amazonDeviceType,
           gadgetAmazonDeviceType);
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].additionalIdentification.modelName,
           "NordicBLEGadget");
    strcpy(discover_response_event_envelope.event.payload.endpoints[0].additionalIdentification.radioAddress,
           "default");
    bool status = pb_encode(&stream, alexaDiscovery_DiscoverResponseEventProto_fields,
                            &discover_response_event_envelope);
    if (!status)
    {
        free(deviceSecret);
        deviceSecret = NULL;
        DBG_DIRECT("%s: Error encoding message\n", __FUNCTION__);
        return 0;
    }
    //sample discovery response
    //0AE8030A240A0F416C6578612E446973636F766572791211446973636F7665722E526573706F6E73
    //6512BF030ABC030A086E7266353238343012094E6F72646963424C451A084E6F72646963444B5A6C
    //0A0E416C657861496E74657266616365121A416C6578612E4761646765742E53746174654C697374
    //656E65721A03312E3022390A0A0A0874696D65696E666F0A080A0674696D6572730A080A06616C61
    //726D730A0B0A0972656D696E646572730A0A0A0877616B65776F72645A1D0A0E416C657861496E74
    //6572666163651206416C657274731A03312E315A240A0E416C657861496E74657266616365120D4E
    //6F74696669636174696F6E731A03312E305A380A0E416C657861496E746572666163651216416C65
    //78612E4761646765742E4D75736963446174611A03312E3022090A070A0574656D706F5A3A0A0E41
    //6C657861496E746572666163651217416C6578612E4761646765742E537065656368446174611A03
    //312E30220A0A080A06766973656D6562720A01311240393746423630323742354141353046354431
    //43424642304442423941383833314433334342463941344542363633444346344539313139393845
    //3330373534381A0131220E413157454F433630594F314A56562A0F4E6F72646963424C4547616467
    //6574320764656661756C74
    DBG_DIRECT("bytes written:%zu\n", stream.bytes_written);
    free(deviceSecret);
    deviceSecret = NULL;
    return (uint16_t)stream.bytes_written;
}

void gadget_discovery_request(AMAData *data)
{
    DBG_DIRECT("********************************************\n");
    DBG_DIRECT("***********   DISCOVERY REQUEST   **********\n");
    DBG_DIRECT("********************************************\n");

    pb_istream_t stream_discovery = pb_istream_from_buffer(data->data, data->length);
    pb_decode(&stream_discovery, alexaDiscovery_DiscoverDirectiveProto_fields,
              &discover_directive_envelope);
    DBG_DIRECT("scope type: %d", discover_directive_envelope.directive.payload.scope.type);
    DBG_DIRECT("scope token: %d", discover_directive_envelope.directive.payload.scope.token);
    uint16_t encoded_length = encode_sample_discover_response_event();

    DBG_DIRECT("Post encoding. Buffer size received: %d\n", encoded_length);

    AMAData *tx_data = malloc(sizeof(AMAData) + encoded_length);
    tx_data->device = data->device;
    tx_data->stream = data->stream;
    tx_data->length = encoded_length;
    memcpy(tx_data->data, &buffer, encoded_length);
    gadget_send_data(tx_data);
    free(tx_data);
}

void gadget_set_alert(AMAData *data)
{
    DBG_DIRECT("********************************************\n");
    DBG_DIRECT("*************  ALERTS SETALERT  ************\n");
    DBG_DIRECT("********************************************\n");

    // Parse the set alert payload now
    pb_istream_t stream_setalert = pb_istream_from_buffer(data->data, data->length);
    pb_decode(&stream_setalert, alerts_SetAlertDirectiveProto_fields, &setalert_envelope);
    //DBG_DIRECT("%s",setalert_envelope.directive.payload.scheduledTime);
    isTimerExist = true;
}

void gadget_delete_alert(AMAData *data)
{
    DBG_DIRECT("********************************************\n");
    DBG_DIRECT("************  ALERTS DELETEALERT  **********\n");
    DBG_DIRECT("********************************************\n");
    stop_led_flash();
    // Parse the delete alert payload now
    pb_istream_t stream_deletealert = pb_istream_from_buffer(data->data, data->length);
    pb_decode(&stream_deletealert, alerts_DeleteAlertDirectiveProto_fields, &deletealert_envelope);
}

void gadget_notification_set_indicator(AMAData *data)
{
    DBG_DIRECT("********************************************\n");
    DBG_DIRECT("********  NOTIFICATION SETINDICATOR  *******\n");
    DBG_DIRECT("********************************************\n");

    // Parse the notification payload now
    pb_istream_t stream_setnotification = pb_istream_from_buffer(data->data, data->length);
    pb_decode(&stream_setnotification, notifications_SetIndicatorDirectiveProto_fields,
              &setnotifications_envelope);
    DBG_DIRECT("visualIndicator:%d, audioIndicator=%d, assetId=%s, url=%s\n",
               setnotifications_envelope.directive.payload.persistVisualIndicator,
               setnotifications_envelope.directive.payload.playAudioIndicator,
               setnotifications_envelope.directive.payload.asset.assetId,
               setnotifications_envelope.directive.payload.asset.url);
}

void gadget_notification_clear_indicator(AMAData *data)
{
    DBG_DIRECT("********************************************\n");
    DBG_DIRECT("*******  NOTIFICATION CLEARINDICATOR  ******\n");
    DBG_DIRECT("********************************************\n");

    // Parse the notification payload now
    pb_istream_t stream_clearnotification = pb_istream_from_buffer(data->data, data->length);
    pb_decode(&stream_clearnotification, notifications_ClearIndicatorDirectiveProto_fields,
              &clearnotifications_envelope);
}

void gadget_state_listener_state_update(AMAData *data)
{
    DBG_DIRECT("********************************************\n");
    DBG_DIRECT("******   STATE LISTENER STATE UPDATE   *****\n");
    DBG_DIRECT("********************************************\n");

    // Parse the state update payload now
    pb_istream_t stateupdate_stream = pb_istream_from_buffer(data->data, data->length);
    pb_decode(&stateupdate_stream, alexaGadgetStateListener_StateUpdateDirectiveProto_fields,
              &stateupdate_envelope);
    int states_count = stateupdate_envelope.directive.payload.states_count;
    for (int i = 0; i < states_count; ++i)
    {
        DBG_DIRECT("state name: %s, state value: %s\n",
                   stateupdate_envelope.directive.payload.states[i].name,
                   stateupdate_envelope.directive.payload.states[i].value);
        if ((0 == strcmp(stateupdate_envelope.directive.payload.states[i].name, "timers")) &&
            (0 == strcmp(stateupdate_envelope.directive.payload.states[i].value, "active")))
        {
            start_led_flash();
            DBG_DIRECT("GADGET: Alert Start");
        }
    }
}

void gadget_speech_data_speech_marks(AMAData *data)
{
    DBG_DIRECT("********************************************\n");
    DBG_DIRECT("**********  SPEECHDATA SPEECHMARKS  ********\n");
    DBG_DIRECT("********************************************\n");

    // Parse the speechmarks payload now
    pb_istream_t speechmarks_stream = pb_istream_from_buffer(data->data, data->length);
    pb_decode(&speechmarks_stream, alexaGadgetSpeechData_SpeechmarksDirectiveProto_fields,
              &speechmarks_envelope);
    int speechmarks_count = speechmarks_envelope.directive.payload.speechmarksData_count;
    DBG_DIRECT("player offset: %d\n",
               speechmarks_envelope.directive.payload.playerOffsetInMilliSeconds);
    for (int i = 0; i < speechmarks_count; ++i)
    {
        DBG_DIRECT("speechmark type: %s\n", speechmarks_envelope.directive.payload.speechmarksData[i].type);
        DBG_DIRECT("speechmark value: %s\n",
                   speechmarks_envelope.directive.payload.speechmarksData[i].value);
        DBG_DIRECT("speechmark start offset: %d\n",
                   speechmarks_envelope.directive.payload.speechmarksData[i].startOffsetInMilliSeconds);
    }
}

void gadget_music_data_tempo(AMAData *data)
{
    DBG_DIRECT("********************************************\n");
    DBG_DIRECT("************   MUSICDATA TEMPO   ***********\n");
    DBG_DIRECT("********************************************\n");

    // Parse the tempo payload now
    pb_istream_t tempo_stream = pb_istream_from_buffer(data->data, data->length);
    pb_decode(&tempo_stream, alexaGadgetMusicData_TempoDirectiveProto_fields, &tempo_envelope);
    DBG_DIRECT("Tempo Envelope Payload Tempo Value: %s\n",
               tempo_envelope.directive.payload.tempoData[0].value);
}

void gadget_received_data(AMAData *data)
{
    DBG_DIRECT("Received Stream [%d] Length [%d]\n", data->stream, data->length);
    pb_istream_t stream = pb_istream_from_buffer(data->data, data->length);
    pb_decode(&stream, directive_DirectiveParserProto_fields, &directive_parser_envelope);
    // https://github.com/alexa/alexa-gadgets-sample-code/blob/master/AlexaGadgetsProtobuf/examples/proto_sample.c

    if (0 == strcmp(directive_parser_envelope.directive.header.name, "Discover") &&
        (0 == strcmp(directive_parser_envelope.directive.header.namespace, "Alexa.Discovery")))
    {
        gadget_discovery_request(data);
    }
    else if (0 == strcmp(directive_parser_envelope.directive.header.name, "SetAlert") &&
             (0 == strcmp(directive_parser_envelope.directive.header.namespace, "Alerts")))
    {
        gadget_set_alert(data);
    }
    else if (0 == strcmp(directive_parser_envelope.directive.header.name, "DeleteAlert") &&
             (0 == strcmp(directive_parser_envelope.directive.header.namespace, "Alerts")))
    {
        gadget_delete_alert(data);
    }
    else if (0 == strcmp(directive_parser_envelope.directive.header.name, "SetIndicator") &&
             (0 == strcmp(directive_parser_envelope.directive.header.namespace, "Notifications")))
    {
        gadget_notification_set_indicator(data);
    }
    else if (0 == strcmp(directive_parser_envelope.directive.header.name, "ClearIndicator") &&
             (0 == strcmp(directive_parser_envelope.directive.header.namespace, "Notifications")))
    {
        gadget_notification_clear_indicator(data);
    }
    else if (0 == strcmp(directive_parser_envelope.directive.header.name, "StateUpdate") &&
             (0 == strcmp(directive_parser_envelope.directive.header.namespace, "Alexa.Gadget.StateListener")))
    {
        gadget_state_listener_state_update(data);
    }
    else if (0 == strcmp(directive_parser_envelope.directive.header.name, "Speechmarks") &&
             (0 == strcmp(directive_parser_envelope.directive.header.namespace, "Alexa.Gadget.SpeechData")))
    {
        gadget_speech_data_speech_marks(data);
    }
    else if (0 == strcmp(directive_parser_envelope.directive.header.name, "Tempo") &&
             (0 == strcmp(directive_parser_envelope.directive.header.namespace, "Alexa.Gadget.MusicData")))
    {
        gadget_music_data_tempo(data);
    }
    else
    {
        DBG_DIRECT("********************************************\n");
        DBG_DIRECT("**************   UNSUPPORTED   *************\n");
        DBG_DIRECT("Header name: %s\n", directive_parser_envelope.directive.header.name);
        DBG_DIRECT("Header namespace: %s\n", directive_parser_envelope.directive.header.namespace);
        DBG_DIRECT("********************************************\n");
        DBG_DIRECT("Error: do not have parsing code for this directive, check directive name\n");
    }
}
