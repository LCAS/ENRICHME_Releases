time                  Begin
string                TypeCD
time                  End
string                Name
string                srcDomain
string                srcId
uint8                 Level
string                CentreCD
string                TargetCD
time                  Shown
string                Up
string                EventName
string                Ext
string                Handling
uint8                 Relevance
string                Surname
string                ConclusionCD
string                ID
string                Conclusion

#NCP event description.
#
# Fields according to Events Rev02.docx and changed according to what I got from GET methods
# string field lengths (where known) are checked! 
#
#Field        Data type        Validity                 Description
#
#Begin        Date           POST then read-only        Date & time of start of the event.
#TypeCD       String(4)      POST then read-only        Type of event code.
#End          Date           POST, PUT, single-time     Date & time of end of the event.
#Name         String(?)
#srcDomain    String(7)      POST then read-only        Static value to “ULINAAL” for the AAL server.
#srcId        String(64)     POST then read-only        Universal ID identifying the event. It should identify the installation of the AAL server and must be unique among all the installations of the AAL srv.
#Level        Integer(?)
#CentreCD     String(?)
#TargetCD     String(15)     POST then read-only        Target code (internal code of the patient). Must be on system
#Shown        Date           Read-only                  Date & time of end of when the event has been shown to an user.
#Up           String(?)
#EventName    String(?)
#Ext          XML            POST then read-only        Extended information about the event.
#Handling     String(?)
#Relevance    Integer(?)
#Surname      String(?)
#ConclusionCD String(?)
#ID           UUID           Read-only                  The ID of the event. Given by NCP
#Conclusion   String(?)

