#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "msg_types__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__msg_types__msg__FaceDetect() -> *const std::ffi::c_void;
}

#[link(name = "msg_types__rosidl_generator_c")]
extern "C" {
    fn msg_types__msg__FaceDetect__init(msg: *mut FaceDetect) -> bool;
    fn msg_types__msg__FaceDetect__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FaceDetect>, size: usize) -> bool;
    fn msg_types__msg__FaceDetect__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FaceDetect>);
    fn msg_types__msg__FaceDetect__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FaceDetect>, out_seq: *mut rosidl_runtime_rs::Sequence<FaceDetect>) -> bool;
}

// Corresponds to msg_types__msg__FaceDetect
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FaceDetect {

    // This member is not documented.
    #[allow(missing_docs)]
    pub x: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub id: i8,

}



impl Default for FaceDetect {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !msg_types__msg__FaceDetect__init(&mut msg as *mut _) {
        panic!("Call to msg_types__msg__FaceDetect__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FaceDetect {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { msg_types__msg__FaceDetect__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { msg_types__msg__FaceDetect__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { msg_types__msg__FaceDetect__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FaceDetect {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FaceDetect where Self: Sized {
  const TYPE_NAME: &'static str = "msg_types/msg/FaceDetect";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__msg_types__msg__FaceDetect() }
  }
}


#[link(name = "msg_types__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__msg_types__msg__SubjectClear() -> *const std::ffi::c_void;
}

#[link(name = "msg_types__rosidl_generator_c")]
extern "C" {
    fn msg_types__msg__SubjectClear__init(msg: *mut SubjectClear) -> bool;
    fn msg_types__msg__SubjectClear__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SubjectClear>, size: usize) -> bool;
    fn msg_types__msg__SubjectClear__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SubjectClear>);
    fn msg_types__msg__SubjectClear__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SubjectClear>, out_seq: *mut rosidl_runtime_rs::Sequence<SubjectClear>) -> bool;
}

// Corresponds to msg_types__msg__SubjectClear
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SubjectClear {

    // This member is not documented.
    #[allow(missing_docs)]
    pub id: i8,

}



impl Default for SubjectClear {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !msg_types__msg__SubjectClear__init(&mut msg as *mut _) {
        panic!("Call to msg_types__msg__SubjectClear__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SubjectClear {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { msg_types__msg__SubjectClear__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { msg_types__msg__SubjectClear__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { msg_types__msg__SubjectClear__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SubjectClear {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SubjectClear where Self: Sized {
  const TYPE_NAME: &'static str = "msg_types/msg/SubjectClear";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__msg_types__msg__SubjectClear() }
  }
}


#[link(name = "msg_types__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__msg_types__msg__RingDetect() -> *const std::ffi::c_void;
}

#[link(name = "msg_types__rosidl_generator_c")]
extern "C" {
    fn msg_types__msg__RingDetect__init(msg: *mut RingDetect) -> bool;
    fn msg_types__msg__RingDetect__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RingDetect>, size: usize) -> bool;
    fn msg_types__msg__RingDetect__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RingDetect>);
    fn msg_types__msg__RingDetect__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RingDetect>, out_seq: *mut rosidl_runtime_rs::Sequence<RingDetect>) -> bool;
}

// Corresponds to msg_types__msg__RingDetect
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RingDetect {

    // This member is not documented.
    #[allow(missing_docs)]
    pub x: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub id: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub color: rosidl_runtime_rs::String,

}



impl Default for RingDetect {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !msg_types__msg__RingDetect__init(&mut msg as *mut _) {
        panic!("Call to msg_types__msg__RingDetect__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RingDetect {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { msg_types__msg__RingDetect__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { msg_types__msg__RingDetect__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { msg_types__msg__RingDetect__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RingDetect {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RingDetect where Self: Sized {
  const TYPE_NAME: &'static str = "msg_types/msg/RingDetect";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__msg_types__msg__RingDetect() }
  }
}


