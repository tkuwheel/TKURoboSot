#!/usr/bin/env python
#
# This file is part of libfreespace.
#  
# Copyright (c) 2009-2010 Hillcrest Laboratories, Inc. 
# 
# libfreespace is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
# 
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

import sys
import argparse
import os

def compareMessages(a, b):
    # Sort ID[0] before ID[1] before ID[2]
    if len(a.ID[0]) != 0:
        if len(b.ID[0]) != 0:
            return cmp(a.ID[0]['constID'], b.ID[0]['constID'])
        else:
            return -1
    elif len(a.ID[1]) !=0:
        if len(b.ID[0]) != 0:
            return 1
        elif len(b.ID[1]) != 0:
            return cmp(a.ID[1]['constID'], b.ID[1]['constID'])
        else:
            return -1
    elif len(a.ID[2]) != 0:
        if len(b.ID[0]) != 0:
            return 1
        elif len(b.ID[1]) != 0:
            return 1
        elif len(b.ID[2]) != 0:
            return cmp(a.ID[2]['constID'], b.ID[2]['constID'])
        else:
            return -1
    else:
        if len(b.ID[2]) != 0:
            return 1
        else:
            return 0

class MessageCodeGenerator:

    inclDir = ""
    srcDir  = ""

    def __init__(self, incl, src):
        self.inclDir = incl
        self.srcDir = src

    def writeMessages(self, messages):
        messages.sort(compareMessages)


        codecsFileName = "freespace_codecs"
        printersFileName = "freespace_printers"
        codecsHdrPath = os.path.join(self.inclDir, codecsFileName + ".h")
        printerHdrPath = os.path.join(self.inclDir, printersFileName + ".h")
        codecsSrcPath = os.path.join(self.srcDir, codecsFileName + ".c")
        printersSrcPath = os.path.join(self.srcDir, printersFileName + ".c")

        codecsHFile = open(codecsHdrPath, "w")
        self.writeHFileHeader(codecsHFile, codecsFileName)
        self.writeDoxygenModuleDef(codecsHFile)
        
        printersHFile = open(printerHdrPath, "w")
        self.writeHFileHeader(printersHFile, printersFileName)
        printersHFile.write('#include "' + codecsFileName + '.h"\n')
        printersHFile.write('#include <stdio.h>\n\n')
        self.writePrintMessageHeader(printersHFile)
        
        codecsCFile = open(codecsSrcPath, "w")
        self.writeCFileHeader(codecsCFile, codecsFileName)
        codecsCFile.write('#include <stdio.h>\n')
        codecsCFile.write('#include <math.h>\n')
        codecsCFile.write('\n#ifdef _WIN32\n')
        codecsCFile.write('#define STRICT_DECODE_LENGTH 0\n')
        codecsCFile.write('#else\n')
        codecsCFile.write('#define STRICT_DECODE_LENGTH 0\n')
        codecsCFile.write('#endif\n\n')
        codecsCFile.write('#undef CODECS_PRINTF\n')
        codecsCFile.write('//#define CODECS_DEBUG\n')
        codecsCFile.write('#ifdef CODECS_DEBUG\n')
        codecsCFile.write('#define CODECS_PRINTF printf\n')
        codecsCFile.write('#else\n')
        codecsCFile.write('#define CODECS_PRINTF(...)\n')
        codecsCFile.write('#endif\n\n')
        self.writeBitHelper(codecsCFile)
        
        printersCFile = open(printersSrcPath, "w")
        self.writeCFileHeader(printersCFile, printersFileName)
        self.writePrintMessageBody(messages, printersCFile)
        
        for message in messages:
            fields = extractFields(message)
            # Data structure to hold the message
            writeStruct(message, fields, codecsHFile)

        self.writeUnionStruct(codecsHFile, messages)

        for message in messages:
            writeCodecs(message, codecsHFile, codecsCFile)
            writePrinter(message, printersHFile, printersCFile)

        self.writeUnionDecodeEncodeBodies(codecsCFile, messages)
            
        self.writeHFileTrailer(codecsHFile, codecsFileName)
        self.writeHFileTrailer(printersHFile, printersFileName)
        codecsHFile.close()
        codecsCFile.close()
        printersHFile.close()
        printersCFile.close()
    
    def writeBitHelper(self, outHeader):
        outHeader.write('''
static uint32_t toUint32(const uint8_t * a) {
#ifdef FREESPACE_LITTLE_ENDIAN
    return ((((uint32_t) a[3])) << 24) | ((((uint32_t) a[2])) << 16) | ((((uint32_t) a[1])) << 8) | (uint32_t) a[0];
#else
    return ((((uint32_t) a[0])) << 24) | ((((uint32_t) a[1])) << 16) | ((((uint32_t) a[2])) << 8) | (uint32_t) a[3];
#endif
}

static uint16_t toUint16(const uint8_t * a) {
#ifdef FREESPACE_LITTLE_ENDIAN
    return ((((uint16_t) a[1])) << 8) | (uint16_t) a[0];
#else
    return ((((uint16_t) a[0])) << 8) | (uint16_t) a[1];
#endif
}

static uint8_t toUint8(const uint8_t * a) {
    return (uint8_t) *a;
}

static int32_t toInt32(const uint8_t * a) {
#ifdef FREESPACE_LITTLE_ENDIAN
    return (int32_t) (((((int32_t) a[3])) << 24) | ((((uint32_t) a[2])) << 16) | ((((uint32_t) a[1])) << 8) | (uint32_t) a[0]);
#else
    return (int32_t) (((((int32_t) a[0])) << 24) | ((((uint32_t) a[1])) << 16) | ((((uint32_t) a[2])) << 8) | (uint32_t) a[3]);
#endif
}

static int16_t toInt16(const uint8_t * a) {
#ifdef FREESPACE_LITTLE_ENDIAN
    return (((int16_t) a[1]) << 8) | a[0];
#else
    return (((int16_t) a[0]) << 8) | a[1];
#endif
}

static int8_t toInt8(const uint8_t * a) {
    return (int8_t) *a;
}

static uint8_t getBit(uint8_t a, uint16_t whichBit) {
    return (uint8_t) (a >> whichBit) & 0x01;
}

static uint8_t getNibble(uint8_t a, uint16_t whichNibble) {
    return (uint8_t) (a >> (whichNibble*4)) & 0x0F;
}

static uint8_t byteFromNibbles(uint8_t lsn, uint8_t msn) {
    return lsn | (msn << 4);
}


''')

    def writeHFileHeader(self, outHeader, name):
        # Print the struct, and define the message functions
        writeCopyright(outHeader)
        outHeader.write("\n")
        writeIfndef(outHeader, name)
        outHeader.write("\n")
        outHeader.write('#include "freespace/freespace_common.h"\n')
        outHeader.write("\n")
        writeExternC(outHeader)
        outHeader.write('\n')
        
    def writeHFileTrailer(self, outHeader, name):
        writeCloseExternC(outHeader)
        outHeader.write("\n")
        writeCloseIfndef(outHeader, name)

    def writeCFileHeader(self, outFile, name):
        writeCopyright(outFile)
        outFile.write('''
#include "freespace/%s.h"
#include <string.h>
'''%name)
        
    def writePrintMessageBody(self, messages, outFile):
        outFile.write('''
static void printUnknown(const char* name, const uint8_t* buffer, int length) {
    int i;
    printf("%s(", name);
    for (i = 0; i < length; ++i) {
        printf("%02x ", (uint8_t) buffer[i]);
    }
    printf(")\\n");
}

int freespace_printMessageStr(char* dest, int maxlen, const struct freespace_message* s) {
    switch(s->messageType) {''')
        for message in messages:
            #if (not message.decode):
            #    continue
            outFile.write('''
    case %(enumName)s:
        return freespace_print%(messageName)sStr(dest, maxlen, &(s->%(structName)s));
        break;'''%{'enumName':message.enumName, 
                   'messageName':message.name, 
                   'structName':message.structName})
        outFile.write('''
    default:
        return -1;
    }
}

void freespace_printMessage(FILE* fp, const struct freespace_message * s) {
    char buf[1024];
    int rc = freespace_printMessageStr(buf, sizeof(buf), s);
    
    if (rc < 0) {
        fprintf(fp, "invalid messages\\n");
        return;
    }
    fprintf(fp, "%s\\n", buf);
}

''')
    
    def writePrintMessageHeader(self, outFile):
        outFile.write('''
/**
 * Pretty print a Freespace message to the terminal.
 *
 * @param fp the file pointer to print into
 * @param s the HID message
 */

LIBFREESPACE_API void freespace_printMessage(FILE* fp, const struct freespace_message * s);

/**
 * Pretty print message struct to string dest, with maximum length maxlen.
 * @param dest the destination string
 * @param maxlen the length of the passed in string
 * @param s the struct to print
 * @return the number of characters actually printed, or an error if it tries to print more than maxlen
 */
LIBFREESPACE_API int freespace_printMessageStr(char* dest, int maxlen, const struct freespace_message* s);

''')
    
    def writeDoxygenModuleDef(self, outFile):
        outFile.write('''
/**
 * @defgroup messages Freespace Messages
 *
 * This page describes the messages that can be sent to and from the Freespace Device.
 * They are represented as structs which can be encoded and decoded from strings.
 */
''')
            
    def writeUnionStruct(self, file, messages):
        file.write('''
/** @ingroup messages
 * An enumeration for all the types of messages that can exist. Used in freespace_message
 * to determine the type of message contained in the union
 */
enum MessageTypes {''')
        i = 0
        for message in messages:
            file.write('''
    %s = %d,'''%(message.enumName, i))
            i = i+1
        file.write('''
};
''')
    
        file.write('''
/** @ingroup messages
 * freespace_message has an enum which defines the type of the message contained
 * and a union of all the possible message structs. 
 * When these structs are allocated, they must be memset to all 0 before assigning any fields.
 */
struct freespace_message {
    int messageType;
    uint8_t ver;  /**< HID protocol version */
    uint8_t len;  /**< Length, used in version 2 only */
    uint8_t dest; /**< Destination, used in version 2 only */
    uint8_t src;  /**< Source, used in version 2 only */

    union {''')
        for message in messages:
            file.write("\n\t\tstruct freespace_%(name)s %(varName)s;"%{'name':message.name, 'varName':message.structName})
        file.write('''
    };
};

/** @ingroup messages
 * Decode an arbitrary message. Fill out the corresponding values in struct s.
 *
 * @param message the message to decode that was received from the Freespace device
 * @param length the length of the received message
 * @param s the preallocated freespace_message struct to decode into
 * @param ver the HID protocol version to use to decode the message
 * @return FREESPACE_SUCESS or an error code
 */
LIBFREESPACE_API int freespace_decode_message(const uint8_t* message, int length, struct freespace_message* s, uint8_t ver);

/** @ingroup messages
 * Encode an arbitrary message.
 *
 * @param message the freespace_message struct
 * @param msgBuf the buffer to put the encoded message into
 * @param maxLength the maximum length of the encoded message (i.e sizeof(*msgBuf))
 * @return the actual size of the encoded message or an error code
 */
LIBFREESPACE_API int freespace_encode_message(struct freespace_message* message, uint8_t* msgBuf, int maxLength);

''')

    def writeUnionDecodeEncodeBodies(self, file, messages):
        file.write('''
LIBFREESPACE_API int freespace_decode_message(const uint8_t* message, int length, struct freespace_message* s, uint8_t ver) {
    if (length == 0) {
        return -1;
    }

    switch (ver) {\n''')
        subIdMap = [1, 1, 4] # A lookup table that tells where in the message to find the sub ID. The HID version is the index to the table.
        for v in range(3):
            usedIDs = []
            file.write("\t\tcase %d:\n" % v)
            file.write("\t\t\tswitch(message[0]) {\n")
            for message in messages:
                if (not message.decode) or len(message.ID[v]) == 0:
                    continue
                if message.ID[v]['constID'] in usedIDs:
                    continue
                file.write("\t\t\t\tcase %d:"%message.ID[v]['constID'])
                if message.ID[v].has_key('subId'):
                    file.write('''
                    switch (message[%d]) {''' % subIdMap[v])
                    for subMessage in messages:
                        if (not subMessage.decode) or len(subMessage.ID[v]) == 0:
                            continue
                        if subMessage.ID[v]['constID'] == message.ID[v]['constID']:
                            file.write('''
                        case %(subId)d:
                            s->messageType = %(messageType)s;
                            return freespace_decode%(subName)s(message, length, s, ver);'''
                            %{'subId':subMessage.ID[v]['subId']['id'],
                           'subName':subMessage.name,
                           'messageType':subMessage.enumName})                
            
                    file.write('''
                        default:
                            return FREESPACE_ERROR_MALFORMED_MESSAGE;
                    }\n''')
                else:
                    file.write('''
                    s->messageType = %(messageType)s;
                    return freespace_decode%(name)s(message, length, s, ver);
'''%{'messageType':message.enumName,
                     'name':message.name})
                    
                usedIDs.append(message.ID[v]['constID'])
            file.write('''                default:
                    return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }\n''')
        file.write('''
    default:
        return FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;
    }
}
''')
        
        file.write('''
LIBFREESPACE_API int freespace_encode_message(struct freespace_message* message, uint8_t* msgBuf, int maxlength) {
    message->src = 0; // Force source to 0, since this is coming from the system host.
    switch (message->messageType) {''')
        for message in messages:
            if (not message.encode):
                continue
            file.write('''
        case %(enumName)s:
            return freespace_encode%(messageName)s(message, msgBuf, maxlength);'''%{'enumName':message.enumName, 
                                                                                    'messageName':message.name})
        file.write('''
        default:
            return -1;
        }
}''')



# --------------------------  Individual Message ------------------------------------
    
def writeCodecs(message, outHFile, outCFile):
    fields = extractFields(message)
    writeCodecHeader(message, fields, outHFile)
    writeCodecCFile(message, fields, outCFile)
            
def writePrinter(message, outHFile, outCFile):
    writePrinterHeader(message, outHFile)
    writePrinterBody(message, outCFile)
    
def writePrinterHeader(message, outHeader):
    writePrintDecl(message, outHeader)
    
def writePrinterBody(message, outFile):
    writePrintBody(message, outFile)
    outFile.write('\n')
    
# Add an entry to the codec header file for one message
def writeCodecHeader(message, fields, outHeader):
    # Decode function declaration
    if message.decode:
        writeDecodeDecl(message, outHeader)
        outHeader.write('\n')
    # Encode function declaration
    if message.encode:
        writeEncodeDecl(message, outHeader)
        outHeader.write('\n')

# Add an entry to the codec file to encode or decode one message
def writeCodecCFile(message, fields, outFile):
    if message.decode:
        writeDecodeBody(message, fields, outFile)
        outFile.write('\n')

    if message.encode:
        writeEncodeBody(message, fields, outFile)
        outFile.write('\n')

def writeStruct(message, fields, outHeader):
    outHeader.write("\n")
    if message.Documentation != None:
        outHeader.write("/** @ingroup messages \n * " + message.Documentation + "\n */\n")
    outHeader.write("struct freespace_" + message.name + " {\n")
    if len(fields) > 0:
        for field in fields:
            if len(field['Doc']):
                outHeader.write("\n\t/** " + field['Doc'] + " */\n")
            outHeader.write("\t")
            outHeader.write(field['type'])
            outHeader.write(" " + field['name'])
            if field['count'] != 1:
                outHeader.write("[%d]"%field['count'])
            outHeader.write(";\n")
    else:
        outHeader.write("\tuint8_t nothing; // This is here to keep the compiler happy.\n")
    outHeader.write("};\n")
        
def extractFields(message):
    fields = {}
    fieldsList = []
    for version in message.Fields:
        for field in version:
            if field['name'] == 'RESERVED':
                continue
            if field.has_key('cType'):
                if not fields.has_key(field['name']):
                    item = cTypeToTypeInfo(field['cType'], field['size'])
                    if field.has_key('comment'):
                        item['Doc'] = field['comment']
                    else:
                        item['Doc'] = ""
                    if item['warning'] == 'yes':
                        print ("Type problem found in message=>%s, field=>%s"%(message.name, field['name']))
                    item['name'] = field['name']
                    field['typeDecode'] = item
                    fields[field['name']] = item
                    fieldsList.append(item)
                else:
                    field['typeDecode'] = fields[field['name']]
            elif field.has_key('bits'):
                for bit in field['bits']:
                    if bit['name'] == 'RESERVED':
                        continue
                    if not fields.has_key(bit['name']):
                        item = bitToTypeInfo(bit)
                        if bit.has_key('comment'):
                            item['Doc'] = bit['comment']
                        else:
                            item['Doc'] = ""
                        item['name'] = bit['name']
                        bit['typeDecode'] = item
                        fields[bit['name']] = item
                        fieldsList.append(item)
                    else:
                        bit['typeDecode'] = fields[bit['name']]
            elif field.has_key('nibbles'):
                for nibble in field['nibbles']:
                    if nibble['name'] == 'RESERVED':
                        continue
                    if not fields.has_key(nibble['name']):
                        item = {'type':'int', 'signed':False, 'length':4, 'count':1, 'warning':'no'}
                        if nibble.has_key('comment'):
                            item['Doc'] = nibble['comment']
                        else:
                            item['Doc'] = ""
                        item['name'] = nibble['name']
                        nibble['typeDecode'] = item
                        fields[nibble['name']] = item
                        fieldsList.append(item)
                    else:
                        nibble['typeDecode'] = fields[nibble['name']]
    return fieldsList
    
def cTypeToTypeInfo(ct, sizeInBytes):
    typeInfo = {'type':ct, 'warning':'no'}
    if ct == 'uint32_t':
        typeInfo['signed'] = False
        typeInfo['count'] = sizeInBytes / 4
        typeInfo['width'] = 4
    elif ct == 'uint16_t':
        typeInfo['signed'] = False
        typeInfo['count'] = sizeInBytes / 2
        typeInfo['width'] = 2
    elif ct == 'uint8_t':
        typeInfo['signed'] = False
        typeInfo['count'] = sizeInBytes / 1
        typeInfo['width'] = 1
    elif ct == 'int32_t':
        typeInfo['signed'] = True
        typeInfo['count'] = sizeInBytes / 4
        typeInfo['width'] = 4
    elif ct == 'int16_t':
        typeInfo['signed'] = True
        typeInfo['count'] = sizeInBytes / 2
        typeInfo['width'] = 2
    elif ct == 'int8_t':
        typeInfo['signed'] = True
        typeInfo['count'] = sizeInBytes / 1
        typeInfo['width'] = 1
    else:
        print("ERROR: Unrecognized cType: %s"%ct)
        sys.exit(1)
    if sizeInBytes != typeInfo['count'] * typeInfo['width']:
        print("WARNING: Size(%d) is not a multiple of the size of cType(%s)"%(sizeInBytes, ct))
        typeInfo['warning'] = 'yes'
    return typeInfo
    
def bitToTypeInfo(bt):
    if not bt.has_key('size'):
        return {'type':'uint8_t', 'signed':False, 'length':1, 'count':1, 'warning':'no'}
    elif bt['size'] == 1:
        return {'type':'uint8_t', 'signed':False, 'length':1, 'count':1, 'warning':'no'}
    else:
        return {'type':'int', 'signed':False, 'length':bt['size'], 'count':1, 'warning':'no'}
    
def writeEncodeDecl(message, outHeader):
    outHeader.write('''
/** @ingroup messages
 * Encode a %(name)s message.
 *
 * @param m the freespace_message struct to encode
 * @param message the string to put the encoded message into
 * @param maxlength the maximum length of the message
 * @return the actual size of the encoded message or an error code
 */
LIBFREESPACE_API int freespace_encode%(name)s(const struct freespace_message* m, uint8_t* message, int maxlength);
'''%{'name':message.name})
    
def writeDecodeDecl(message, outHeader):
    outHeader.write('''
/** @ingroup messages
 * Decode a %(name)s message. Fill out the corresponding values in struct s.
 *
 * @param message the message to decode that was received from the Freespace device
 * @param length the length of the received message
 * @param m the preallocated freespace_message struct into which to decode
 * @param ver the protocol version to use for this message
 * @return FREESPACE_SUCCESS or an error
 */
LIBFREESPACE_API int freespace_decode%(name)s(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver);
'''%{'name':message.name})

def writePrintDecl(message, outHeader):
    outHeader.write('''
/**
 * Print message struct to string dest, with maximum length maxlen.
 * @param dest the destination string
 * @param maxlen the length of the passed in string
 * @param s the struct to print
 * @return the number of characters actually printed, or an error if it tries to print more than maxlen
 */
LIBFREESPACE_API int freespace_print%(name)sStr(char* dest, int maxlen, const struct freespace_%(name)s* s);
/**
 * Print message to a file pointer.
 * @param fp the destination file pointer
 * @param s the struct to print
 * @return the number of characters actually printed, or an error if it tries to print more than maxlen
 */
LIBFREESPACE_API int freespace_print%(name)s(FILE* fp, const struct freespace_%(name)s* s);

'''%{'name':message.name})
    
def writeEncodeBody(message, fields, outFile):
    
    outFile.write("LIBFREESPACE_API int freespace_encode%s(const struct freespace_message* m, uint8_t* message, int maxlength) {\n"%message.name)
        
    outFile.write("\n\tuint8_t offset = 1;\n")
    
    if len(fields) > 0:
        outFile.write("\tconst struct freespace_%s* s = &(m->%s);\n\n"%(message.name, message.structName))
    
    # Encode switch statement
    outFile.write("\tswitch(m->ver) {\n")
    for v in range(3):
        byteCounter = 0
        if len(message.ID[v]):
            # Create one case per version of message
            outFile.write("\t\tcase %d:\n"%v)
            # Check message buffer length
            outFile.write("\t\t\tif (maxlength < %d) {\n"%message.getMessageSize(v))
            outFile.write('\t\t\t\tCODECS_PRINTF("freespace_%s encode(<INVALID LENGTH>)\\n");\n'%message.name)
            outFile.write('\t\t\t\treturn FREESPACE_ERROR_BUFFER_TOO_SMALL;\n')
            outFile.write("\t\t\t}\n")
            # Message ID
            outFile.write("\t\t\tmessage[0] = (uint8_t) %d;\n"%message.ID[v]['constID'])
            # dest and src fields in version 2 messages
            if v == 2:
                outFile.write("\t\t\tmessage[2] = m->dest;\n")
                outFile.write("\t\t\tmessage[3] = m->src;\n")
                outFile.write("\t\t\toffset = 4;\n")
            # Message sub ID, if defined
            if message.ID[v].has_key('subId'):
                outFile.write("\t\t\tmessage[%d + offset] = (uint8_t) %d;\n" % (byteCounter, message.ID[v]['subId']['id']))
                byteCounter += 1

            # Message fields
            for field in message.Fields[v]:
                if field.has_key('synthesized'):
                    continue
                elementSize = field['size']
                if field['name'] == 'RESERVED':
                    byteCounter += elementSize
                    continue
                if field.has_key('bits'):
                    bitoffset = 0
                    exprs = []
                    for bit in field['bits']:
                        sz = bit.get('size', 1);
                        if bit['name'] != 'RESERVED':
                            mask = (1 << sz) - 1
                            exprs.append('((s->%s & 0x%x) << %d)' % (bit['name'], mask, bitoffset))
                        bitoffset += sz
                    exprs = "\n\t\t\t\t\t\t\t\t|  ".join(exprs)
                    outFile.write('\t\t\tmessage[%d + offset] = (%s);\n' % (byteCounter, exprs))
                    byteCounter += 1
                elif field.has_key('nibbles'):
                    outFile.write('\t\t\tmessage[%d + offset] = byteFromNibbles('%byteCounter)
                    firstLoop = True
                    for nibble in field['nibbles']:
                        if not firstLoop:
                            outFile.write(', ')
                        else:
                            firstLoop = False
                        if nibble['name'] == 'RESERVED':
                            outFile.write('0')
                        else:
                            outFile.write('s->%s'%nibble['name']);
                    outFile.write(');\n')
                    byteCounter += 1
                elif field.has_key('cType'):
                    if field['typeDecode']['count'] == 1:
                        for j in range (field['typeDecode']['width']):
                            outFile.write('\t\t\tmessage[%d + offset] = s->%s >> %d;\n'%(byteCounter, field['name'], 8 * j))
                            byteCounter += 1
                    else:
                        for i in range (field['typeDecode']['count']):
                            for j in range (field['typeDecode']['width']):
                                outFile.write('\t\t\tmessage[%d + offset] = s->%s[%d] >> %d;\n'%(byteCounter, field['name'], i, 8 * j))
                                byteCounter += 1
                else:
                    print ("Unrecognized field type in %s\n" % message.name)
            if v == 2:
                outFile.write("\t\t\tmessage[1] = %d + offset;\n" % byteCounter)
            outFile.write("\t\t\treturn %d + offset;\n" % byteCounter)
    # Default case
    outFile.write("\t\tdefault:\n")
    outFile.write("\t\t\treturn  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;\n")
    outFile.write("\t}")
        
    # End of function
    outFile.write('\r}\n')

def writeDecodeBody(message, fields, outFile):
    outFile.write("LIBFREESPACE_API int freespace_decode%s(const uint8_t* message, int length, struct freespace_message* m, uint8_t ver) {" %message.name)
    outFile.write("\n\tuint8_t offset = 1;\n")
    if len(fields) > 0:
        outFile.write("\tstruct freespace_%s* s = &(m->%s);\n\n"%(message.name, message.structName))
    outFile.write("\tm->ver = ver;\n\n")
    # Encode switch statement
    outFile.write("\tswitch(ver) {\n")
    byteCounter = 0
    for v in range(3):
        if len(message.ID[v]):
            # Create one case per version of message
            byteCounter = 0
            outFile.write("\t\tcase %d:\n"%v)
            # Code to check message buffer length and report ID
            if len(message.ID[v]):
                outFile.write('''            if ((STRICT_DECODE_LENGTH && length != %(size)d) || (!STRICT_DECODE_LENGTH && length < %(size)d)) {
                CODECS_PRINTF(\"Length mismatch for %%s.  Expected %%d.  Got %%d.\\n\", \"%(name)s\", %(size)d, length);
                return FREESPACE_ERROR_BUFFER_TOO_SMALL;
            }
            if ((uint8_t) message[0] != %(id)d) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
'''%{'size':message.getMessageSize(v), 'id':message.ID[v]['constID'], 'name':message.name})
            if v == 2:
                outFile.write("\t\t\toffset = 4;\n")
                outFile.write("\t\t\tm->len = message[1];\n")
                outFile.write("\t\t\tm->dest = message[2];\n")
                outFile.write("\t\t\tm->src = message[3];\n")

            if message.ID[v].has_key('subId'):
                outFile.write('''
            if ((uint8_t) message[offset] != %d) {
                return FREESPACE_ERROR_MALFORMED_MESSAGE;
            }
'''%message.ID[v]['subId']['id'])
                byteCounter += 1
            for field in message.Fields[v]:
                if field.has_key('synthesized'):
                    continue
                elementSize = field['size']
                if field['name'] == 'RESERVED':
                    byteCounter += elementSize
                    continue
                if field.has_key('cType'):
                    if field['typeDecode']['count'] == 1:
                        outFile.write("\t\t\ts->%s = %s(&message[%d + offset]);\n" % (field['name'], IntConversionHelper(field['typeDecode']['type']), byteCounter))
                        byteCounter += field['typeDecode']['width']
                    else:
                        for i in range (field['typeDecode']['count']):
                            outFile.write("\t\t\ts->%s[%d] = %s(&message[%d + offset]);\n" % (field['name'], i, IntConversionHelper(field['typeDecode']['type']), byteCounter))
                            byteCounter += field['typeDecode']['width']
                elif field.has_key('bits'):
                    bitCounter = 0
                    for bit in field['bits']:
                        if bit['name'] != 'RESERVED':
                            if bit.has_key('size'):
                                outFile.write("\t\t\ts->%s = (uint8_t) ((message[%d + offset] >> %d) & 0x%02X);\n"%(bit['name'], byteCounter, bitCounter, 2**bit['size']-1))
                                bitCounter += bit['size']-1
                            else:
                                outFile.write("\t\t\ts->%s = getBit(message[%d + offset], %d);\n"%(bit['name'], byteCounter, bitCounter))
                        bitCounter += 1
                    byteCounter += 1
                elif field.has_key('nibbles'):

                    nibbleCounter = 0
                    for nibble in field['nibbles']:
                        if nibble['name'] != 'RESERVED':
                            outFile.write('    s->%s = getNibble(message[%d + offset], %d);\n'%(nibble['name'], byteCounter, nibbleCounter))
                        nibbleCounter += 1
                    byteCounter += 1
                else:
                    print ("Unrecognized field type in %s\n" % message.name)
            for field in message.Fields[v]:
                if field.has_key('synthesized'):
                    outFile.write(specialCaseCode(field['synthesized']))
            outFile.write("\t\t\treturn FREESPACE_SUCCESS;\n")
    # Default case
    outFile.write("\t\tdefault:\n")
    outFile.write("\t\t\treturn  FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION;\n")
    outFile.write('\t}\n')
    outFile.write('}\n')

def printStrHelper(message, outFile):
    fields = extractFields(message)
    first = True
    for field in fields:
        if not first:
            outFile.write(' ')
        else:
            first = False
        if field['count'] == 1:
            outFile.write("%s=%%d"%field['name'])
    outFile.write(')"')
    for field in fields:
        if field['count'] == 1:
            outFile.write(', s->%s'%field['name'])
    outFile.write(');')

def writePrintBody(message, outFile):
    outFile.write('''
LIBFREESPACE_API int freespace_print%(name)sStr(char* dest, int maxlen, const struct freespace_%(name)s* s) {
    int n;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
#ifdef _WIN32
    n = sprintf_s(dest, maxlen, "%(name)s('''%{'name':message.name})
    printStrHelper(message, outFile)
    outFile.write('''
#else
    n = snprintf(dest, maxlen, "%(name)s('''%{'name':message.name})
    printStrHelper(message, outFile)
    
    outFile.write('''
#endif
    if (n < 0) {
        return FREESPACE_ERROR_BUFFER_TOO_SMALL;
    }
    return n;
}
''')
    
    
    outFile.write('''
LIBFREESPACE_API int freespace_print%(name)s(FILE* fp, const struct freespace_%(name)s* s) {
    char str[1024];
    int rc;
    if (s == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    rc = freespace_print%(name)sStr(str, sizeof(str), s);
    if (rc < 0) {
        return rc;
    }
    return fprintf(fp, "%%s\\n", str);
}
'''%{'name':message.name})
        
# --------------------------  Syntax Helpers ------------------------------------

def fieldToPrintFormat(field):
    return '%d'

def IntConversionHelper(type):
    intConverters = {'uint32_t':"toUint32",
    'uint16_t':"toUint16",
    'uint8_t':"toUint8",
    'int32_t':"toInt32",
    'int16_t':"toInt16",
    'int8_t':"toInt8"}
    return intConverters[type]
    
# ---------------------- Output printing helpers --------------------------------

def writeCopyright(outHeader):
    outHeader.write('''/*
 * This file is part of libfreespace.
 * 
 * Copyright (c) 2009-2012 Hillcrest Laboratories, Inc. 
 *
 * libfreespace is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */
''')

def writeIfndef(outHeader, name):
    outHeader.write("#ifndef " + name.upper() + "_H_\n")
    outHeader.write("#define " + name.upper() + "_H_\n")
    
def writeCloseIfndef(outHeader, name):
    outHeader.write("#endif /* " + name.upper() + "_H_ */\n")
    
def writeExternC(outHeader):
    outHeader.write("#ifdef __cplusplus\n")
    outHeader.write('extern "C" {\n')
    outHeader.write('#endif\n')
    
def writeCloseExternC(outHeader):
    outHeader.write("#ifdef __cplusplus\n")
    outHeader.write('}\n')
    outHeader.write('#endif\n')

#----------------------- Special Case Code ----------------------------
def specialCaseCode(case):
    if case == 'case_A':
        # Calculate the A value of the quaternion
        # A = sqrt(16384**2 - (B**2 + C**2 + D**2))
        specialCode = "\t\t\ts->angularPosA = (int16_t) sqrt(268435456 - ((s->angularPosB * s->angularPosB) + (s->angularPosC * s->angularPosC) + (s->angularPosD * s->angularPosD)));\n"
    else:
        print ("Unrecognized special case: %s" % case)
        specialCode =  "Unknown code goes here."
        
    return specialCode
# ---------------------- Main function --------------------------------
# Courtesy of Guido: http://www.artima.com/weblogs/viewpost.jsp?thread=4829
class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg


def main(argv=None):
    if argv is None:
        argv = sys.argv



    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("-t", "--test", 
                            default=False, 
                            help="Use test mode to build BOTH encoders and decoders for ALL message types. " +
                                 "By default encoders are built for outgoing msgs and decoders for incoming msgs")
        parser.add_argument("-I", "--include", default="include", 
                            help="Include directory to write generated freespace headers to")
        parser.add_argument("-s", "--src", default="src",
                            help="Source directory to write generated source files to")
        parser.add_argument("messageFiles", nargs="+",
                            help="List of message definition files")
        args = parser.parse_args()

        messages = []
        g = {'test': args.test}
        d = {}
        
        for f in args.messageFiles:
            execfile(f, g, d)
            messages.extend(d['messages'])

        includeDir = os.path.join(args.include, "freespace")
        srcDir = args.src

        for d in (includeDir, srcDir):
            if not os.path.exists(d):
                os.makedirs(d)

        mcg = MessageCodeGenerator(
            includeDir,
            srcDir
        )
        mcg.writeMessages(messages)
    except Usage, err:
        print >>sys.stderr, err.msg
        print >>sys.stderr, "for help use --help"
        return 2
        
        
if __name__ == "__main__":
    __doc__ = "Options\n"
    __doc__ = __doc__ + "\t--h\t--help\tHelp menu\n"
    __doc__ = __doc__ + "\t--t\t--test\tGenerate encoders and decoders for all messages\n"
    __doc__ = __doc__ + "\n"
    __doc__ = __doc__ + "Arguments\n"
    __doc__ = __doc__ + "\t<file>\t*.py\tGenerate codecs & printers for messages in file"
    sys.exit(main())

