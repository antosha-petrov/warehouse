using WarehouseApp.ViewModel;

namespace WarehouseApp;

public partial class OrderPage : ContentPage
{
	public OrderPage()
	{
		InitializeComponent();
        BindingContext = new OrderPageViewModel();
    }
}