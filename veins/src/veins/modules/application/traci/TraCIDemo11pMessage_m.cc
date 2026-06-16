//
// Generated file, do not edit! Created by opp_msgtool 6.1 from veins/modules/application/traci/TraCIDemo11pMessage.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wshadow"
#  pragma clang diagnostic ignored "-Wconversion"
#  pragma clang diagnostic ignored "-Wunused-parameter"
#  pragma clang diagnostic ignored "-Wc++98-compat"
#  pragma clang diagnostic ignored "-Wunreachable-code-break"
#  pragma clang diagnostic ignored "-Wold-style-cast"
#elif defined(__GNUC__)
#  pragma GCC diagnostic ignored "-Wshadow"
#  pragma GCC diagnostic ignored "-Wconversion"
#  pragma GCC diagnostic ignored "-Wunused-parameter"
#  pragma GCC diagnostic ignored "-Wold-style-cast"
#  pragma GCC diagnostic ignored "-Wsuggest-attribute=noreturn"
#  pragma GCC diagnostic ignored "-Wfloat-conversion"
#endif

#include <iostream>
#include <sstream>
#include <memory>
#include <type_traits>
#include "TraCIDemo11pMessage_m.h"

namespace omnetpp {

// Template pack/unpack rules. They are declared *after* a1l type-specific pack functions for multiple reasons.
// They are in the omnetpp namespace, to allow them to be found by argument-dependent lookup via the cCommBuffer argument

// Packing/unpacking an std::vector
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::vector<T,A>& v)
{
    int n = v.size();
    doParsimPacking(buffer, n);
    for (int i = 0; i < n; i++)
        doParsimPacking(buffer, v[i]);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::vector<T,A>& v)
{
    int n;
    doParsimUnpacking(buffer, n);
    v.resize(n);
    for (int i = 0; i < n; i++)
        doParsimUnpacking(buffer, v[i]);
}

// Packing/unpacking an std::list
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::list<T,A>& l)
{
    doParsimPacking(buffer, (int)l.size());
    for (typename std::list<T,A>::const_iterator it = l.begin(); it != l.end(); ++it)
        doParsimPacking(buffer, (T&)*it);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::list<T,A>& l)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i = 0; i < n; i++) {
        l.push_back(T());
        doParsimUnpacking(buffer, l.back());
    }
}

// Packing/unpacking an std::set
template<typename T, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::set<T,Tr,A>& s)
{
    doParsimPacking(buffer, (int)s.size());
    for (typename std::set<T,Tr,A>::const_iterator it = s.begin(); it != s.end(); ++it)
        doParsimPacking(buffer, *it);
}

template<typename T, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::set<T,Tr,A>& s)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i = 0; i < n; i++) {
        T x;
        doParsimUnpacking(buffer, x);
        s.insert(x);
    }
}

// Packing/unpacking an std::map
template<typename K, typename V, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::map<K,V,Tr,A>& m)
{
    doParsimPacking(buffer, (int)m.size());
    for (typename std::map<K,V,Tr,A>::const_iterator it = m.begin(); it != m.end(); ++it) {
        doParsimPacking(buffer, it->first);
        doParsimPacking(buffer, it->second);
    }
}

template<typename K, typename V, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::map<K,V,Tr,A>& m)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i = 0; i < n; i++) {
        K k; V v;
        doParsimUnpacking(buffer, k);
        doParsimUnpacking(buffer, v);
        m[k] = v;
    }
}

// Default pack/unpack function for arrays
template<typename T>
void doParsimArrayPacking(omnetpp::cCommBuffer *b, const T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimPacking(b, t[i]);
}

template<typename T>
void doParsimArrayUnpacking(omnetpp::cCommBuffer *b, T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimUnpacking(b, t[i]);
}

// Default rule to prevent compiler from choosing base class' doParsimPacking() function
template<typename T>
void doParsimPacking(omnetpp::cCommBuffer *, const T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: No doParsimPacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

template<typename T>
void doParsimUnpacking(omnetpp::cCommBuffer *, T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: No doParsimUnpacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

}  // namespace omnetpp

namespace veins {

Register_Class(TraCIDemo11pMessage)

TraCIDemo11pMessage::TraCIDemo11pMessage(const char *name, short kind) : ::veins::BaseFrame1609_4(name, kind)
{
}

TraCIDemo11pMessage::TraCIDemo11pMessage(const TraCIDemo11pMessage& other) : ::veins::BaseFrame1609_4(other)
{
    copy(other);
}

TraCIDemo11pMessage::~TraCIDemo11pMessage()
{
}

TraCIDemo11pMessage& TraCIDemo11pMessage::operator=(const TraCIDemo11pMessage& other)
{
    if (this == &other) return *this;
    ::veins::BaseFrame1609_4::operator=(other);
    copy(other);
    return *this;
}

void TraCIDemo11pMessage::copy(const TraCIDemo11pMessage& other)
{
    this->demoData = other.demoData;
    this->senderAddress = other.senderAddress;
    this->serial = other.serial;
    this->Vehicle_ID = other.Vehicle_ID;
    this->Frame_ID = other.Frame_ID;
    this->Total_Frames = other.Total_Frames;
    this->Global_Time = other.Global_Time;
    this->Local_X = other.Local_X;
    this->Local_Y = other.Local_Y;
    this->Global_X = other.Global_X;
    this->Global_Y = other.Global_Y;
    this->v_Length = other.v_Length;
    this->v_Width = other.v_Width;
    this->v_Class = other.v_Class;
    this->v_Vel = other.v_Vel;
    this->v_Acc = other.v_Acc;
    this->Lane_ID = other.Lane_ID;
    this->Preceding = other.Preceding;
    this->Following = other.Following;
    this->Space_Headway = other.Space_Headway;
    this->Time_Headway = other.Time_Headway;
}

void TraCIDemo11pMessage::parsimPack(omnetpp::cCommBuffer *b) const
{
    ::veins::BaseFrame1609_4::parsimPack(b);
    doParsimPacking(b,this->demoData);
    doParsimPacking(b,this->senderAddress);
    doParsimPacking(b,this->serial);
    doParsimPacking(b,this->Vehicle_ID);
    doParsimPacking(b,this->Frame_ID);
    doParsimPacking(b,this->Total_Frames);
    doParsimPacking(b,this->Global_Time);
    doParsimPacking(b,this->Local_X);
    doParsimPacking(b,this->Local_Y);
    doParsimPacking(b,this->Global_X);
    doParsimPacking(b,this->Global_Y);
    doParsimPacking(b,this->v_Length);
    doParsimPacking(b,this->v_Width);
    doParsimPacking(b,this->v_Class);
    doParsimPacking(b,this->v_Vel);
    doParsimPacking(b,this->v_Acc);
    doParsimPacking(b,this->Lane_ID);
    doParsimPacking(b,this->Preceding);
    doParsimPacking(b,this->Following);
    doParsimPacking(b,this->Space_Headway);
    doParsimPacking(b,this->Time_Headway);
}

void TraCIDemo11pMessage::parsimUnpack(omnetpp::cCommBuffer *b)
{
    ::veins::BaseFrame1609_4::parsimUnpack(b);
    doParsimUnpacking(b,this->demoData);
    doParsimUnpacking(b,this->senderAddress);
    doParsimUnpacking(b,this->serial);
    doParsimUnpacking(b,this->Vehicle_ID);
    doParsimUnpacking(b,this->Frame_ID);
    doParsimUnpacking(b,this->Total_Frames);
    doParsimUnpacking(b,this->Global_Time);
    doParsimUnpacking(b,this->Local_X);
    doParsimUnpacking(b,this->Local_Y);
    doParsimUnpacking(b,this->Global_X);
    doParsimUnpacking(b,this->Global_Y);
    doParsimUnpacking(b,this->v_Length);
    doParsimUnpacking(b,this->v_Width);
    doParsimUnpacking(b,this->v_Class);
    doParsimUnpacking(b,this->v_Vel);
    doParsimUnpacking(b,this->v_Acc);
    doParsimUnpacking(b,this->Lane_ID);
    doParsimUnpacking(b,this->Preceding);
    doParsimUnpacking(b,this->Following);
    doParsimUnpacking(b,this->Space_Headway);
    doParsimUnpacking(b,this->Time_Headway);
}

const char * TraCIDemo11pMessage::getDemoData() const
{
    return this->demoData.c_str();
}

void TraCIDemo11pMessage::setDemoData(const char * demoData)
{
    this->demoData = demoData;
}

const LAddress::L2Type& TraCIDemo11pMessage::getSenderAddress() const
{
    return this->senderAddress;
}

void TraCIDemo11pMessage::setSenderAddress(const LAddress::L2Type& senderAddress)
{
    this->senderAddress = senderAddress;
}

int TraCIDemo11pMessage::getSerial() const
{
    return this->serial;
}

void TraCIDemo11pMessage::setSerial(int serial)
{
    this->serial = serial;
}

int TraCIDemo11pMessage::getVehicle_ID() const
{
    return this->Vehicle_ID;
}

void TraCIDemo11pMessage::setVehicle_ID(int Vehicle_ID)
{
    this->Vehicle_ID = Vehicle_ID;
}

int TraCIDemo11pMessage::getFrame_ID() const
{
    return this->Frame_ID;
}

void TraCIDemo11pMessage::setFrame_ID(int Frame_ID)
{
    this->Frame_ID = Frame_ID;
}

int TraCIDemo11pMessage::getTotal_Frames() const
{
    return this->Total_Frames;
}

void TraCIDemo11pMessage::setTotal_Frames(int Total_Frames)
{
    this->Total_Frames = Total_Frames;
}

double TraCIDemo11pMessage::getGlobal_Time() const
{
    return this->Global_Time;
}

void TraCIDemo11pMessage::setGlobal_Time(double Global_Time)
{
    this->Global_Time = Global_Time;
}

double TraCIDemo11pMessage::getLocal_X() const
{
    return this->Local_X;
}

void TraCIDemo11pMessage::setLocal_X(double Local_X)
{
    this->Local_X = Local_X;
}

double TraCIDemo11pMessage::getLocal_Y() const
{
    return this->Local_Y;
}

void TraCIDemo11pMessage::setLocal_Y(double Local_Y)
{
    this->Local_Y = Local_Y;
}

double TraCIDemo11pMessage::getGlobal_X() const
{
    return this->Global_X;
}

void TraCIDemo11pMessage::setGlobal_X(double Global_X)
{
    this->Global_X = Global_X;
}

double TraCIDemo11pMessage::getGlobal_Y() const
{
    return this->Global_Y;
}

void TraCIDemo11pMessage::setGlobal_Y(double Global_Y)
{
    this->Global_Y = Global_Y;
}

double TraCIDemo11pMessage::getV_Length() const
{
    return this->v_Length;
}

void TraCIDemo11pMessage::setV_Length(double v_Length)
{
    this->v_Length = v_Length;
}

double TraCIDemo11pMessage::getV_Width() const
{
    return this->v_Width;
}

void TraCIDemo11pMessage::setV_Width(double v_Width)
{
    this->v_Width = v_Width;
}

int TraCIDemo11pMessage::getV_Class() const
{
    return this->v_Class;
}

void TraCIDemo11pMessage::setV_Class(int v_Class)
{
    this->v_Class = v_Class;
}

double TraCIDemo11pMessage::getV_Vel() const
{
    return this->v_Vel;
}

void TraCIDemo11pMessage::setV_Vel(double v_Vel)
{
    this->v_Vel = v_Vel;
}

double TraCIDemo11pMessage::getV_Acc() const
{
    return this->v_Acc;
}

void TraCIDemo11pMessage::setV_Acc(double v_Acc)
{
    this->v_Acc = v_Acc;
}

const char * TraCIDemo11pMessage::getLane_ID() const
{
    return this->Lane_ID.c_str();
}

void TraCIDemo11pMessage::setLane_ID(const char * Lane_ID)
{
    this->Lane_ID = Lane_ID;
}

int TraCIDemo11pMessage::getPreceding() const
{
    return this->Preceding;
}

void TraCIDemo11pMessage::setPreceding(int Preceding)
{
    this->Preceding = Preceding;
}

int TraCIDemo11pMessage::getFollowing() const
{
    return this->Following;
}

void TraCIDemo11pMessage::setFollowing(int Following)
{
    this->Following = Following;
}

double TraCIDemo11pMessage::getSpace_Headway() const
{
    return this->Space_Headway;
}

void TraCIDemo11pMessage::setSpace_Headway(double Space_Headway)
{
    this->Space_Headway = Space_Headway;
}

double TraCIDemo11pMessage::getTime_Headway() const
{
    return this->Time_Headway;
}

void TraCIDemo11pMessage::setTime_Headway(double Time_Headway)
{
    this->Time_Headway = Time_Headway;
}

class TraCIDemo11pMessageDescriptor : public omnetpp::cClassDescriptor
{
  private:
    mutable const char **propertyNames;
    enum FieldConstants {
        FIELD_demoData,
        FIELD_senderAddress,
        FIELD_serial,
        FIELD_Vehicle_ID,
        FIELD_Frame_ID,
        FIELD_Total_Frames,
        FIELD_Global_Time,
        FIELD_Local_X,
        FIELD_Local_Y,
        FIELD_Global_X,
        FIELD_Global_Y,
        FIELD_v_Length,
        FIELD_v_Width,
        FIELD_v_Class,
        FIELD_v_Vel,
        FIELD_v_Acc,
        FIELD_Lane_ID,
        FIELD_Preceding,
        FIELD_Following,
        FIELD_Space_Headway,
        FIELD_Time_Headway,
    };
  public:
    TraCIDemo11pMessageDescriptor();
    virtual ~TraCIDemo11pMessageDescriptor();

    virtual bool doesSupport(omnetpp::cObject *obj) const override;
    virtual const char **getPropertyNames() const override;
    virtual const char *getProperty(const char *propertyName) const override;
    virtual int getFieldCount() const override;
    virtual const char *getFieldName(int field) const override;
    virtual int findField(const char *fieldName) const override;
    virtual unsigned int getFieldTypeFlags(int field) const override;
    virtual const char *getFieldTypeString(int field) const override;
    virtual const char **getFieldPropertyNames(int field) const override;
    virtual const char *getFieldProperty(int field, const char *propertyName) const override;
    virtual int getFieldArraySize(omnetpp::any_ptr object, int field) const override;
    virtual void setFieldArraySize(omnetpp::any_ptr object, int field, int size) const override;

    virtual const char *getFieldDynamicTypeString(omnetpp::any_ptr object, int field, int i) const override;
    virtual std::string getFieldValueAsString(omnetpp::any_ptr object, int field, int i) const override;
    virtual void setFieldValueAsString(omnetpp::any_ptr object, int field, int i, const char *value) const override;
    virtual omnetpp::cValue getFieldValue(omnetpp::any_ptr object, int field, int i) const override;
    virtual void setFieldValue(omnetpp::any_ptr object, int field, int i, const omnetpp::cValue& value) const override;

    virtual const char *getFieldStructName(int field) const override;
    virtual omnetpp::any_ptr getFieldStructValuePointer(omnetpp::any_ptr object, int field, int i) const override;
    virtual void setFieldStructValuePointer(omnetpp::any_ptr object, int field, int i, omnetpp::any_ptr ptr) const override;
};

Register_ClassDescriptor(TraCIDemo11pMessageDescriptor)

TraCIDemo11pMessageDescriptor::TraCIDemo11pMessageDescriptor() : omnetpp::cClassDescriptor(omnetpp::opp_typename(typeid(veins::TraCIDemo11pMessage)), "veins::BaseFrame1609_4")
{
    propertyNames = nullptr;
}

TraCIDemo11pMessageDescriptor::~TraCIDemo11pMessageDescriptor()
{
    delete[] propertyNames;
}

bool TraCIDemo11pMessageDescriptor::doesSupport(omnetpp::cObject *obj) const
{
    return dynamic_cast<TraCIDemo11pMessage *>(obj)!=nullptr;
}

const char **TraCIDemo11pMessageDescriptor::getPropertyNames() const
{
    if (!propertyNames) {
        static const char *names[] = {  nullptr };
        omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
        const char **baseNames = base ? base->getPropertyNames() : nullptr;
        propertyNames = mergeLists(baseNames, names);
    }
    return propertyNames;
}

const char *TraCIDemo11pMessageDescriptor::getProperty(const char *propertyName) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    return base ? base->getProperty(propertyName) : nullptr;
}

int TraCIDemo11pMessageDescriptor::getFieldCount() const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    return base ? 21+base->getFieldCount() : 21;
}

unsigned int TraCIDemo11pMessageDescriptor::getFieldTypeFlags(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldTypeFlags(field);
        field -= base->getFieldCount();
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,    // FIELD_demoData
        0,    // FIELD_senderAddress
        FD_ISEDITABLE,    // FIELD_serial
        FD_ISEDITABLE,    // FIELD_Vehicle_ID
        FD_ISEDITABLE,    // FIELD_Frame_ID
        FD_ISEDITABLE,    // FIELD_Total_Frames
        FD_ISEDITABLE,    // FIELD_Global_Time
        FD_ISEDITABLE,    // FIELD_Local_X
        FD_ISEDITABLE,    // FIELD_Local_Y
        FD_ISEDITABLE,    // FIELD_Global_X
        FD_ISEDITABLE,    // FIELD_Global_Y
        FD_ISEDITABLE,    // FIELD_v_Length
        FD_ISEDITABLE,    // FIELD_v_Width
        FD_ISEDITABLE,    // FIELD_v_Class
        FD_ISEDITABLE,    // FIELD_v_Vel
        FD_ISEDITABLE,    // FIELD_v_Acc
        FD_ISEDITABLE,    // FIELD_Lane_ID
        FD_ISEDITABLE,    // FIELD_Preceding
        FD_ISEDITABLE,    // FIELD_Following
        FD_ISEDITABLE,    // FIELD_Space_Headway
        FD_ISEDITABLE,    // FIELD_Time_Headway
    };
    return (field >= 0 && field < 21) ? fieldTypeFlags[field] : 0;
}

const char *TraCIDemo11pMessageDescriptor::getFieldName(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldName(field);
        field -= base->getFieldCount();
    }
    static const char *fieldNames[] = {
        "demoData",
        "senderAddress",
        "serial",
        "Vehicle_ID",
        "Frame_ID",
        "Total_Frames",
        "Global_Time",
        "Local_X",
        "Local_Y",
        "Global_X",
        "Global_Y",
        "v_Length",
        "v_Width",
        "v_Class",
        "v_Vel",
        "v_Acc",
        "Lane_ID",
        "Preceding",
        "Following",
        "Space_Headway",
        "Time_Headway",
    };
    return (field >= 0 && field < 21) ? fieldNames[field] : nullptr;
}

int TraCIDemo11pMessageDescriptor::findField(const char *fieldName) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    int baseIndex = base ? base->getFieldCount() : 0;
    if (strcmp(fieldName, "demoData") == 0) return baseIndex + 0;
    if (strcmp(fieldName, "senderAddress") == 0) return baseIndex + 1;
    if (strcmp(fieldName, "serial") == 0) return baseIndex + 2;
    if (strcmp(fieldName, "Vehicle_ID") == 0) return baseIndex + 3;
    if (strcmp(fieldName, "Frame_ID") == 0) return baseIndex + 4;
    if (strcmp(fieldName, "Total_Frames") == 0) return baseIndex + 5;
    if (strcmp(fieldName, "Global_Time") == 0) return baseIndex + 6;
    if (strcmp(fieldName, "Local_X") == 0) return baseIndex + 7;
    if (strcmp(fieldName, "Local_Y") == 0) return baseIndex + 8;
    if (strcmp(fieldName, "Global_X") == 0) return baseIndex + 9;
    if (strcmp(fieldName, "Global_Y") == 0) return baseIndex + 10;
    if (strcmp(fieldName, "v_Length") == 0) return baseIndex + 11;
    if (strcmp(fieldName, "v_Width") == 0) return baseIndex + 12;
    if (strcmp(fieldName, "v_Class") == 0) return baseIndex + 13;
    if (strcmp(fieldName, "v_Vel") == 0) return baseIndex + 14;
    if (strcmp(fieldName, "v_Acc") == 0) return baseIndex + 15;
    if (strcmp(fieldName, "Lane_ID") == 0) return baseIndex + 16;
    if (strcmp(fieldName, "Preceding") == 0) return baseIndex + 17;
    if (strcmp(fieldName, "Following") == 0) return baseIndex + 18;
    if (strcmp(fieldName, "Space_Headway") == 0) return baseIndex + 19;
    if (strcmp(fieldName, "Time_Headway") == 0) return baseIndex + 20;
    return base ? base->findField(fieldName) : -1;
}

const char *TraCIDemo11pMessageDescriptor::getFieldTypeString(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldTypeString(field);
        field -= base->getFieldCount();
    }
    static const char *fieldTypeStrings[] = {
        "string",    // FIELD_demoData
        "veins::LAddress::L2Type",    // FIELD_senderAddress
        "int",    // FIELD_serial
        "int",    // FIELD_Vehicle_ID
        "int",    // FIELD_Frame_ID
        "int",    // FIELD_Total_Frames
        "double",    // FIELD_Global_Time
        "double",    // FIELD_Local_X
        "double",    // FIELD_Local_Y
        "double",    // FIELD_Global_X
        "double",    // FIELD_Global_Y
        "double",    // FIELD_v_Length
        "double",    // FIELD_v_Width
        "int",    // FIELD_v_Class
        "double",    // FIELD_v_Vel
        "double",    // FIELD_v_Acc
        "string",    // FIELD_Lane_ID
        "int",    // FIELD_Preceding
        "int",    // FIELD_Following
        "double",    // FIELD_Space_Headway
        "double",    // FIELD_Time_Headway
    };
    return (field >= 0 && field < 21) ? fieldTypeStrings[field] : nullptr;
}

const char **TraCIDemo11pMessageDescriptor::getFieldPropertyNames(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldPropertyNames(field);
        field -= base->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

const char *TraCIDemo11pMessageDescriptor::getFieldProperty(int field, const char *propertyName) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldProperty(field, propertyName);
        field -= base->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

int TraCIDemo11pMessageDescriptor::getFieldArraySize(omnetpp::any_ptr object, int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldArraySize(object, field);
        field -= base->getFieldCount();
    }
    TraCIDemo11pMessage *pp = omnetpp::fromAnyPtr<TraCIDemo11pMessage>(object); (void)pp;
    switch (field) {
        default: return 0;
    }
}

void TraCIDemo11pMessageDescriptor::setFieldArraySize(omnetpp::any_ptr object, int field, int size) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldArraySize(object, field, size);
            return;
        }
        field -= base->getFieldCount();
    }
    TraCIDemo11pMessage *pp = omnetpp::fromAnyPtr<TraCIDemo11pMessage>(object); (void)pp;
    switch (field) {
        default: throw omnetpp::cRuntimeError("Cannot set array size of field %d of class 'TraCIDemo11pMessage'", field);
    }
}

const char *TraCIDemo11pMessageDescriptor::getFieldDynamicTypeString(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldDynamicTypeString(object,field,i);
        field -= base->getFieldCount();
    }
    TraCIDemo11pMessage *pp = omnetpp::fromAnyPtr<TraCIDemo11pMessage>(object); (void)pp;
    switch (field) {
        default: return nullptr;
    }
}

std::string TraCIDemo11pMessageDescriptor::getFieldValueAsString(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldValueAsString(object,field,i);
        field -= base->getFieldCount();
    }
    TraCIDemo11pMessage *pp = omnetpp::fromAnyPtr<TraCIDemo11pMessage>(object); (void)pp;
    switch (field) {
        case FIELD_demoData: return oppstring2string(pp->getDemoData());
        case FIELD_senderAddress: return "";
        case FIELD_serial: return long2string(pp->getSerial());
        case FIELD_Vehicle_ID: return long2string(pp->getVehicle_ID());
        case FIELD_Frame_ID: return long2string(pp->getFrame_ID());
        case FIELD_Total_Frames: return long2string(pp->getTotal_Frames());
        case FIELD_Global_Time: return double2string(pp->getGlobal_Time());
        case FIELD_Local_X: return double2string(pp->getLocal_X());
        case FIELD_Local_Y: return double2string(pp->getLocal_Y());
        case FIELD_Global_X: return double2string(pp->getGlobal_X());
        case FIELD_Global_Y: return double2string(pp->getGlobal_Y());
        case FIELD_v_Length: return double2string(pp->getV_Length());
        case FIELD_v_Width: return double2string(pp->getV_Width());
        case FIELD_v_Class: return long2string(pp->getV_Class());
        case FIELD_v_Vel: return double2string(pp->getV_Vel());
        case FIELD_v_Acc: return double2string(pp->getV_Acc());
        case FIELD_Lane_ID: return oppstring2string(pp->getLane_ID());
        case FIELD_Preceding: return long2string(pp->getPreceding());
        case FIELD_Following: return long2string(pp->getFollowing());
        case FIELD_Space_Headway: return double2string(pp->getSpace_Headway());
        case FIELD_Time_Headway: return double2string(pp->getTime_Headway());
        default: return "";
    }
}

void TraCIDemo11pMessageDescriptor::setFieldValueAsString(omnetpp::any_ptr object, int field, int i, const char *value) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldValueAsString(object, field, i, value);
            return;
        }
        field -= base->getFieldCount();
    }
    TraCIDemo11pMessage *pp = omnetpp::fromAnyPtr<TraCIDemo11pMessage>(object); (void)pp;
    switch (field) {
        case FIELD_demoData: pp->setDemoData((value)); break;
        case FIELD_serial: pp->setSerial(string2long(value)); break;
        case FIELD_Vehicle_ID: pp->setVehicle_ID(string2long(value)); break;
        case FIELD_Frame_ID: pp->setFrame_ID(string2long(value)); break;
        case FIELD_Total_Frames: pp->setTotal_Frames(string2long(value)); break;
        case FIELD_Global_Time: pp->setGlobal_Time(string2double(value)); break;
        case FIELD_Local_X: pp->setLocal_X(string2double(value)); break;
        case FIELD_Local_Y: pp->setLocal_Y(string2double(value)); break;
        case FIELD_Global_X: pp->setGlobal_X(string2double(value)); break;
        case FIELD_Global_Y: pp->setGlobal_Y(string2double(value)); break;
        case FIELD_v_Length: pp->setV_Length(string2double(value)); break;
        case FIELD_v_Width: pp->setV_Width(string2double(value)); break;
        case FIELD_v_Class: pp->setV_Class(string2long(value)); break;
        case FIELD_v_Vel: pp->setV_Vel(string2double(value)); break;
        case FIELD_v_Acc: pp->setV_Acc(string2double(value)); break;
        case FIELD_Lane_ID: pp->setLane_ID((value)); break;
        case FIELD_Preceding: pp->setPreceding(string2long(value)); break;
        case FIELD_Following: pp->setFollowing(string2long(value)); break;
        case FIELD_Space_Headway: pp->setSpace_Headway(string2double(value)); break;
        case FIELD_Time_Headway: pp->setTime_Headway(string2double(value)); break;
        default: throw omnetpp::cRuntimeError("Cannot set field %d of class 'TraCIDemo11pMessage'", field);
    }
}

omnetpp::cValue TraCIDemo11pMessageDescriptor::getFieldValue(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldValue(object,field,i);
        field -= base->getFieldCount();
    }
    TraCIDemo11pMessage *pp = omnetpp::fromAnyPtr<TraCIDemo11pMessage>(object); (void)pp;
    switch (field) {
        case FIELD_demoData: return pp->getDemoData();
        case FIELD_senderAddress: return omnetpp::toAnyPtr(&pp->getSenderAddress()); break;
        case FIELD_serial: return pp->getSerial();
        case FIELD_Vehicle_ID: return pp->getVehicle_ID();
        case FIELD_Frame_ID: return pp->getFrame_ID();
        case FIELD_Total_Frames: return pp->getTotal_Frames();
        case FIELD_Global_Time: return pp->getGlobal_Time();
        case FIELD_Local_X: return pp->getLocal_X();
        case FIELD_Local_Y: return pp->getLocal_Y();
        case FIELD_Global_X: return pp->getGlobal_X();
        case FIELD_Global_Y: return pp->getGlobal_Y();
        case FIELD_v_Length: return pp->getV_Length();
        case FIELD_v_Width: return pp->getV_Width();
        case FIELD_v_Class: return pp->getV_Class();
        case FIELD_v_Vel: return pp->getV_Vel();
        case FIELD_v_Acc: return pp->getV_Acc();
        case FIELD_Lane_ID: return pp->getLane_ID();
        case FIELD_Preceding: return pp->getPreceding();
        case FIELD_Following: return pp->getFollowing();
        case FIELD_Space_Headway: return pp->getSpace_Headway();
        case FIELD_Time_Headway: return pp->getTime_Headway();
        default: throw omnetpp::cRuntimeError("Cannot return field %d of class 'TraCIDemo11pMessage' as cValue -- field index out of range?", field);
    }
}

void TraCIDemo11pMessageDescriptor::setFieldValue(omnetpp::any_ptr object, int field, int i, const omnetpp::cValue& value) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldValue(object, field, i, value);
            return;
        }
        field -= base->getFieldCount();
    }
    TraCIDemo11pMessage *pp = omnetpp::fromAnyPtr<TraCIDemo11pMessage>(object); (void)pp;
    switch (field) {
        case FIELD_demoData: pp->setDemoData(value.stringValue()); break;
        case FIELD_serial: pp->setSerial(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_Vehicle_ID: pp->setVehicle_ID(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_Frame_ID: pp->setFrame_ID(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_Total_Frames: pp->setTotal_Frames(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_Global_Time: pp->setGlobal_Time(value.doubleValue()); break;
        case FIELD_Local_X: pp->setLocal_X(value.doubleValue()); break;
        case FIELD_Local_Y: pp->setLocal_Y(value.doubleValue()); break;
        case FIELD_Global_X: pp->setGlobal_X(value.doubleValue()); break;
        case FIELD_Global_Y: pp->setGlobal_Y(value.doubleValue()); break;
        case FIELD_v_Length: pp->setV_Length(value.doubleValue()); break;
        case FIELD_v_Width: pp->setV_Width(value.doubleValue()); break;
        case FIELD_v_Class: pp->setV_Class(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_v_Vel: pp->setV_Vel(value.doubleValue()); break;
        case FIELD_v_Acc: pp->setV_Acc(value.doubleValue()); break;
        case FIELD_Lane_ID: pp->setLane_ID(value.stringValue()); break;
        case FIELD_Preceding: pp->setPreceding(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_Following: pp->setFollowing(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_Space_Headway: pp->setSpace_Headway(value.doubleValue()); break;
        case FIELD_Time_Headway: pp->setTime_Headway(value.doubleValue()); break;
        default: throw omnetpp::cRuntimeError("Cannot set field %d of class 'TraCIDemo11pMessage'", field);
    }
}

const char *TraCIDemo11pMessageDescriptor::getFieldStructName(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldStructName(field);
        field -= base->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    };
}

omnetpp::any_ptr TraCIDemo11pMessageDescriptor::getFieldStructValuePointer(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldStructValuePointer(object, field, i);
        field -= base->getFieldCount();
    }
    TraCIDemo11pMessage *pp = omnetpp::fromAnyPtr<TraCIDemo11pMessage>(object); (void)pp;
    switch (field) {
        case FIELD_senderAddress: return omnetpp::toAnyPtr(&pp->getSenderAddress()); break;
        default: return omnetpp::any_ptr(nullptr);
    }
}

void TraCIDemo11pMessageDescriptor::setFieldStructValuePointer(omnetpp::any_ptr object, int field, int i, omnetpp::any_ptr ptr) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldStructValuePointer(object, field, i, ptr);
            return;
        }
        field -= base->getFieldCount();
    }
    TraCIDemo11pMessage *pp = omnetpp::fromAnyPtr<TraCIDemo11pMessage>(object); (void)pp;
    switch (field) {
        default: throw omnetpp::cRuntimeError("Cannot set field %d of class 'TraCIDemo11pMessage'", field);
    }
}

}  // namespace veins

namespace omnetpp {

}  // namespace omnetpp

